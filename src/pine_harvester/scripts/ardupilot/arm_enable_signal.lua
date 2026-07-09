--[[
arm_enable_signal.lua
ArduPilot Lua 脚本 — 机械臂使能信号控制

功能：
  根据飞行状态自动控制 SERVO9（AUX1）的 PWM 输出，
  作为"机械臂允许伸出"的硬件使能信号发送给 RK3588。

  PWM 1000us = 禁止（机械臂必须收缩）
  PWM 2000us = 允许（机械臂可以伸出）

安装方法：
  1. 将此文件放到 ArduPilot SD 卡的 /APM/scripts/ 目录
  2. 设置 SCR_ENABLE = 1（启用 Lua 脚本引擎）
  3. 设置 SERVO9_FUNCTION = 0（设为手动，由脚本控制）
  4. 重启飞控

参数说明（在脚本顶部修改）：
  SAFE_ALT_CM    最低允许伸臂高度（厘米）
  SERVO_CHANNEL  使能信号的 SERVO 通道号
  PWM_ENABLE     允许伸臂时的 PWM 值（us）
  PWM_DISABLE    禁止伸臂时的 PWM 值（us）

允许伸臂条件（全部满足）：
  1. 飞控已武装（ARMED）
  2. 飞行模式为 GUIDED / LOITER / POSHOLD / ALT_HOLD
  3. 相对高度 > SAFE_ALT_CM

禁止伸臂（任一条件）：
  - 飞控解除武装
  - 飞行模式切换到 LAND / RTL / AUTO（非悬停模式）
  - 高度低于安全高度
  - 脚本出错（fail-safe：默认禁止）
--]]

-- ── 参数配置（按需修改）─────────────────────────────────
local SAFE_ALT_CM    = 150    -- 允许伸臂的最低高度（厘米），默认 1.5m
local SERVO_CHANNEL  = 9      -- SERVO9 = AUX1
local PWM_ENABLE     = 2000   -- 允许伸臂：高电平（us）
local PWM_DISABLE    = 1000   -- 禁止伸臂：低电平（us）
local UPDATE_HZ      = 10     -- 脚本更新频率

-- ── 允许伸臂的飞行模式 ID（Copter）───────────────────────
-- 4=GUIDED, 5=LOITER, 16=POSHOLD, 2=ALT_HOLD
local ALLOWED_MODES = {[4]=true, [5]=true, [16]=true, [2]=true}

-- ── 内部状态 ──────────────────────────────────────────────
local arm_enabled = false
local last_state  = ""

local function set_servo(pwm)
    SRV_Channels:set_output_pwm_chan_timeout(SERVO_CHANNEL - 1, pwm, 1000)
end

local function check_conditions()
    -- 1. 武装检查
    if not arming:is_armed() then
        return false, "DISARMED"
    end

    -- 2. 飞行模式检查
    local mode = vehicle:get_mode()
    if not ALLOWED_MODES[mode] then
        return false, string.format("BAD_MODE(%d)", mode)
    end

    -- 3. 高度检查（相对起飞点）
    local alt_cm = baro:get_altitude() * 100  -- baro 返回米，转厘米
    -- 更准确的方法：用 ahrs:get_relative_position_NED_home()
    local pos = ahrs:get_relative_position_NED_home()
    if pos then
        alt_cm = -pos:z() * 100  -- NED z 取反得到高度
    end

    if alt_cm < SAFE_ALT_CM then
        return false, string.format("LOW_ALT(%.0fcm)", alt_cm)
    end

    return true, "OK"
end

local function update()
    local ok, reason = check_conditions()

    local new_state = string.format("arm_en=%s reason=%s", tostring(ok), reason)

    if ok ~= arm_enabled or new_state ~= last_state then
        arm_enabled = ok
        last_state  = new_state

        if ok then
            set_servo(PWM_ENABLE)
            gcs:send_text(6, "ARM_ENABLE: servo9=" .. PWM_ENABLE .. " (" .. reason .. ")")
        else
            set_servo(PWM_DISABLE)
            gcs:send_text(6, "ARM_DISABLE: servo9=" .. PWM_DISABLE .. " (" .. reason .. ")")
        end
    end

    -- 持续刷新（SRV_Channels timeout 机制需要持续调用）
    if arm_enabled then
        set_servo(PWM_ENABLE)
    else
        set_servo(PWM_DISABLE)
    end

    return update, 1000 / UPDATE_HZ  -- 返回下次调用的毫秒数
end

-- ── 启动时先禁用 ─────────────────────────────────────────
set_servo(PWM_DISABLE)
gcs:send_text(6, "arm_enable_signal.lua loaded | safe_alt=" .. SAFE_ALT_CM .. "cm")

return update, 1000  -- 1s 后开始第一次执行
