import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from rknnlite.api import RKNNLite

    RKNN_OK = True
except ImportError:
    RKNNLite = None
    RKNN_OK = False


PKG_SHARE = get_package_share_directory("pine_harvester")


def _shape_summary(outputs):
    """Return readable RKNN output shapes without assuming a valid tensor."""
    if outputs is None:
        return "None"
    if not isinstance(outputs, (list, tuple)):
        outputs = [outputs]
    return str([tuple(np.asarray(output).shape) for output in outputs])


def _to_prediction_rows(output):
    """Normalize one YOLO output to [num_predictions, num_features].

    Only singleton batch axes are removed.  This deliberately avoids np.squeeze,
    which can turn a one-value result into a scalar.
    """
    array = np.asarray(output)
    if array.size == 0:
        return np.empty((0, 0), dtype=np.float32)

    while array.ndim > 2 and array.shape[0] == 1:
        array = array[0]

    if array.ndim == 1:
        if array.size < 5:
            raise ValueError(
                f"输出只有 {array.size} 个值，无法解析为 YOLOv8 检测结果"
            )
        array = array.reshape(1, -1)
    elif array.ndim != 2:
        raise ValueError(f"不支持的输出维度: {tuple(array.shape)}")

    # YOLOv8 commonly exports [1, 4+C, N], while some converters return
    # [1, N, 4+C].  The feature dimension is the smaller one in normal models.
    rows, columns = array.shape
    if 5 <= rows <= 512 and columns > rows:
        array = array.T

    if array.shape[1] < 5:
        raise ValueError(
            f"每个候选框只有 {array.shape[1]} 个特征，至少需要 4 个坐标和 1 个类别分数"
        )
    return np.asarray(array, dtype=np.float32)


def _collect_prediction_rows(outputs):
    """Collect compatible 2-D prediction tensors returned by RKNN."""
    if outputs is None:
        raise ValueError("RKNN 推理返回 None")
    if not isinstance(outputs, (list, tuple)):
        outputs = [outputs]
    if not outputs:
        raise ValueError("RKNN 推理没有返回任何输出")

    tensors = []
    errors = []
    for output in outputs:
        try:
            rows = _to_prediction_rows(output)
            if rows.size:
                tensors.append(rows)
        except ValueError as exc:
            errors.append(str(exc))

    if not tensors:
        detail = "; ".join(errors) if errors else "输出为空"
        raise ValueError(f"无法解析 RKNN 输出: {detail}")

    feature_counts = {tensor.shape[1] for tensor in tensors}
    if len(feature_counts) != 1:
        raise ValueError(f"多个输出的特征数不一致: {sorted(feature_counts)}")
    return np.concatenate(tensors, axis=0)


def _looks_like_end_to_end(rows):
    """Recognize an end-to-end [x1, y1, x2, y2, score, class] output."""
    if rows.shape[1] != 6 or rows.shape[0] == 0:
        return False
    sample = rows[: min(100, rows.shape[0])]
    finite = np.all(np.isfinite(sample), axis=1)
    sample = sample[finite]
    if sample.size == 0:
        return False
    class_ids = sample[:, 5]
    valid_scores = np.all((sample[:, 4] >= 0.0) & (sample[:, 4] <= 1.0))
    integer_classes = np.all(np.abs(class_ids - np.rint(class_ids)) < 1e-4)
    corner_boxes = np.mean((sample[:, 2] >= sample[:, 0]) & (sample[:, 3] >= sample[:, 1])) > 0.9
    return bool(valid_scores and integer_classes and corner_boxes)


def _decode_yolov8(outputs, image_shape, input_size, conf_threshold):
    """Decode standard YOLOv8 or end-to-end output into OpenCV NMS inputs."""
    rows = _collect_prediction_rows(outputs)
    rows = rows[np.all(np.isfinite(rows), axis=1)]
    if rows.size == 0:
        return [], [], []

    image_height, image_width = image_shape
    input_width, input_height = input_size
    end_to_end = _looks_like_end_to_end(rows)

    if end_to_end:
        scores = rows[:, 4]
        class_ids = np.rint(rows[:, 5]).astype(np.int32)
        keep = scores >= conf_threshold
        coords = rows[keep, :4].copy()
        scores = scores[keep]
        class_ids = class_ids[keep]
        if coords.size == 0:
            return [], [], []

        # End-to-end output stores x1/y1/x2/y2.
        normalized = float(np.max(np.abs(coords))) <= 2.0
        if normalized:
            coords[:, [0, 2]] *= input_width
            coords[:, [1, 3]] *= input_height
        xywh = np.column_stack(
            (coords[:, 0], coords[:, 1], coords[:, 2] - coords[:, 0], coords[:, 3] - coords[:, 1])
        )
    else:
        # Standard YOLOv8 has no separate objectness column:
        # [x_center, y_center, width, height, class_0, ...].
        class_scores = rows[:, 4:]
        class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        keep = scores >= conf_threshold
        xywh_center = rows[keep, :4].copy()
        scores = scores[keep]
        class_ids = class_ids[keep]
        if xywh_center.size == 0:
            return [], [], []

        normalized = float(np.max(np.abs(xywh_center))) <= 2.0
        if normalized:
            xywh_center[:, [0, 2]] *= input_width
            xywh_center[:, [1, 3]] *= input_height
        xywh = np.column_stack(
            (
                xywh_center[:, 0] - xywh_center[:, 2] / 2.0,
                xywh_center[:, 1] - xywh_center[:, 3] / 2.0,
                xywh_center[:, 2],
                xywh_center[:, 3],
            )
        )

    scale_x = image_width / float(input_width)
    scale_y = image_height / float(input_height)
    xywh[:, [0, 2]] *= scale_x
    xywh[:, [1, 3]] *= scale_y

    boxes = []
    final_scores = []
    final_class_ids = []
    for box, score, class_id in zip(xywh, scores, class_ids):
        x, y, width, height = box.tolist()
        x1 = max(0, min(image_width - 1, int(round(x))))
        y1 = max(0, min(image_height - 1, int(round(y))))
        x2 = max(0, min(image_width, int(round(x + width))))
        y2 = max(0, min(image_height, int(round(y + height))))
        if x2 <= x1 or y2 <= y1:
            continue
        # cv2.dnn.NMSBoxes requires [x, y, width, height], not corner points.
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        final_scores.append(float(score))
        final_class_ids.append(int(class_id))
    return boxes, final_scores, final_class_ids


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model_path", os.path.join(PKG_SHARE, "models", "tennis.rknn"))
        self.declare_parameter("debug_viz", True)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)

        self.model_path = self.get_parameter("model_path").value
        self.debug_viz = bool(self.get_parameter("debug_viz").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.bridge = CvBridge()
        self.rknn = None
        self.input_size = (640, 640)
        self._viz_failed = False

        if RKNN_OK:
            try:
                self.rknn = RKNNLite()
                load_result = self.rknn.load_rknn(self.model_path)
                if load_result != 0:
                    raise RuntimeError(f"load_rknn 返回错误码 {load_result}")
                init_result = self.rknn.init_runtime()
                if init_result != 0:
                    raise RuntimeError(f"init_runtime 返回错误码 {init_result}")
                self.get_logger().info(f"RKNN 模型加载成功: {self.model_path}")
            except Exception as exc:
                self.get_logger().error(f"RKNN 模型加载失败: {exc}")
                if self.rknn is not None:
                    self.rknn.release()
                self.rknn = None
        else:
            self.get_logger().warning("RKNNLite 不可用，检测节点不会执行推理")

        self.detections_pub = self.create_publisher(String, "/yolo/detections", 10)
        self.image_sub = self.create_subscription(
            Image, "/camera/body/image_raw", self._image_callback, 10
        )
        self.get_logger().info(f"YOLO 检测器已启动，推理可用: {self.rknn is not None}")

    def _image_callback(self, msg):
        if self.rknn is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image_height, image_width = cv_image.shape[:2]

            resized = cv2.resize(cv_image, self.input_size, interpolation=cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            # RKNN YOLOv8 expects NHWC uint8: (1, 640, 640, 3).
            model_input = np.ascontiguousarray(rgb[np.newaxis, ...], dtype=np.uint8)

            outputs = self.rknn.inference(inputs=[model_input])
            boxes, scores, class_ids = _decode_yolov8(
                outputs,
                (image_height, image_width),
                self.input_size,
                self.conf_threshold,
            )

            selected = []
            if boxes:
                indices = cv2.dnn.NMSBoxes(
                    boxes, scores, self.conf_threshold, self.iou_threshold
                )
                selected = np.asarray(indices, dtype=np.int64).reshape(-1).tolist()

            detections = []
            for index in selected:
                x, y, width, height = boxes[index]
                x2, y2 = x + width, y + height
                detections.append(
                    {
                        "x1": x,
                        "y1": y,
                        "x2": x2,
                        "y2": y2,
                        "score": scores[index],
                        "class_id": class_ids[index],
                    }
                )

                if self.debug_viz and not self._viz_failed:
                    cv2.rectangle(cv_image, (x, y), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        cv_image,
                        f"{class_ids[index]} {scores[index]:.2f}",
                        (x, max(0, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

            self.detections_pub.publish(
                String(data=json.dumps(detections, ensure_ascii=False))
            )

            if self.debug_viz and not self._viz_failed:
                try:
                    cv2.imshow("YOLO Detection", cv_image)
                    cv2.waitKey(1)
                except cv2.error as exc:
                    self._viz_failed = True
                    self.get_logger().warning(f"调试窗口不可用，已自动关闭显示: {exc}")
        except Exception as exc:
            shapes = _shape_summary(locals().get("outputs"))
            self.get_logger().error(f"检测失败，RKNN 输出形状={shapes}: {exc}")

    def destroy_node(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
