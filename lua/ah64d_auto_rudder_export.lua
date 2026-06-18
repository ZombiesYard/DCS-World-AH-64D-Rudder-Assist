-- Optional high-rate telemetry export for ah64d_auto_rudder.
-- Install by adding this line after DCS-BIOS in Saved Games\DCS\Scripts\Export.lua:
-- dofile(lfs.writedir() .. [[Scripts\AH64DAutoRudder\ah64d_auto_rudder_export.lua]])

local AutoRudderExport = AutoRudderExport or {}

AutoRudderExport.host = "127.0.0.1"
AutoRudderExport.port = 34380
AutoRudderExport.rate_hz = 120
AutoRudderExport.probe_rate_hz = 1
AutoRudderExport.last_time = 0
AutoRudderExport.last_probe_time = 0
AutoRudderExport.socket = nil
AutoRudderExport.udp = nil

local function ensure_socket()
    if AutoRudderExport.udp then
        return true
    end

    local ok, socket = pcall(require, "socket")
    if not ok or not socket then
        return false
    end

    local udp = socket.udp()
    if not udp then
        return false
    end

    udp:settimeout(0)
    udp:setpeername(AutoRudderExport.host, AutoRudderExport.port)
    AutoRudderExport.socket = socket
    AutoRudderExport.udp = udp
    return true
end

local function safe_number(value)
    if value == nil then
        return 0
    end
    return value
end

local function find_collective()
    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    if not fm then
        return nil
    end

    local direct_keys = {
        "collective",
        "Collective",
        "collective_position",
        "collectivePosition",
        "mainRotorCollective",
        "main_rotor_collective",
    }
    for _, key in ipairs(direct_keys) do
        if type(fm[key]) == "number" then
            return fm[key]
        end
    end

    for key, value in pairs(fm) do
        if type(value) == "number" and string.find(string.lower(tostring(key)), "collect") then
            return value
        end
    end
    return nil
end

local function find_heading(self_data)
    if self_data and type(self_data.Heading) == "number" then
        return self_data.Heading
    end

    if LoGetADIPitchBankYaw then
        local _, _, yaw = LoGetADIPitchBankYaw()
        if type(yaw) == "number" then
            return yaw
        end
    end

    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    if fm then
        local keys = { "yaw", "Yaw", "heading", "Heading" }
        for _, key in ipairs(keys) do
            if type(fm[key]) == "number" then
                return fm[key]
            end
        end
    end
    return nil
end

local function find_number_by_keys(source, keys)
    if not source then
        return nil
    end
    for _, key in ipairs(keys) do
        if type(source[key]) == "number" then
            return source[key]
        end
    end
    for key, value in pairs(source) do
        local lower_key = string.lower(tostring(key))
        for _, wanted in ipairs(keys) do
            if type(value) == "number" and string.find(lower_key, string.lower(wanted)) then
                return value
            end
        end
    end
    return nil
end

local function optional_number_text(value)
    if type(value) == "number" then
        return string.format("%.8f", value)
    end
    return ""
end

local function vector_number(source, key)
    if type(source) ~= "table" then
        return nil
    end
    local value = source[key]
    if type(value) == "number" then
        return value
    end
    return nil
end

local function vector_speed(x, y, z)
    if type(x) == "number" and type(y) == "number" and type(z) == "number" then
        return math.sqrt(x * x + y * y + z * z)
    end
    return nil
end

local function horizontal_speed_xz(x, z)
    if type(x) == "number" and type(z) == "number" then
        return math.sqrt(x * x + z * z)
    end
    return nil
end

local function average_pair(source, key)
    if not source then
        return nil
    end
    local pair = source[key]
    if type(pair) ~= "table" then
        return nil
    end

    local left = type(pair.left) == "number" and pair.left or nil
    local right = type(pair.right) == "number" and pair.right or nil
    if left and right then
        return (left + right) * 0.5
    end
    return left or right
end

local function average_first_available(source, keys)
    if not source then
        return nil
    end
    for _, key in ipairs(keys) do
        local value = average_pair(source, key)
        if value ~= nil then
            return value
        end
        if type(source[key]) == "number" then
            return source[key]
        end
    end
    return nil
end

local function nested_number(source, keys)
    local current = source
    for _, key in ipairs(keys) do
        if type(current) ~= "table" then
            return nil
        end
        current = current[key]
    end
    if type(current) == "number" then
        return current
    end
    return nil
end

local function cockpit_argument(index)
    if not GetDevice then
        return nil
    end

    local ok_panel, panel = pcall(GetDevice, 0)
    if not ok_panel or not panel or not panel.get_argument_value then
        return nil
    end

    local ok_value, value = pcall(function()
        return panel:get_argument_value(index)
    end)
    if ok_value and type(value) == "number" then
        return value
    end
    return nil
end

local function valid_torque_argument(value)
    return type(value) == "number" and math.abs(value) > 0.001
end

local function probe_key_interesting(source_name, path)
    if source_name == "engine" or source_name == "heli" then
        return true
    end
    local lower = string.lower(path)
    local terms = {
        "tail",
        "anti",
        "antitorque",
        "anti_torque",
        "pedal",
        "rudder",
        "yaw",
        "torque",
        "tq",
        "power",
        "rpm",
        "rotor",
        "engine",
        "eng",
        "nr",
        "np",
        "collect",
        "pitch",
        "bank",
        "roll",
        "speed",
        "velocity",
        "accel",
        "alt",
        "height",
    }
    for _, term in ipairs(terms) do
        if string.find(lower, term, 1, true) then
            return true
        end
    end
    return false
end

local function safe_probe_text(value)
    local text = tostring(value or "")
    text = string.gsub(text, "[,\r\n]", "_")
    return text
end

local function add_probe_numbers(out, source_name, value, prefix, depth)
    if type(value) == "number" then
        if probe_key_interesting(source_name, prefix) then
            out[#out + 1] = {
                source = source_name,
                key = prefix,
                value = value,
            }
        end
        return
    end
    if type(value) ~= "table" or depth >= 4 then
        return
    end
    for key, child in pairs(value) do
        local child_key = tostring(key)
        local child_path = prefix == "" and child_key or (prefix .. "." .. child_key)
        add_probe_numbers(out, source_name, child, child_path, depth + 1)
        if #out >= 160 then
            return
        end
    end
end

local function send_power_probe(now, aircraft)
    if now - AutoRudderExport.last_probe_time < (1 / AutoRudderExport.probe_rate_hz) then
        return
    end
    AutoRudderExport.last_probe_time = now

    local values = {}
    local engine = LoGetEngineInfo and LoGetEngineInfo() or nil
    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    local mech = LoGetMechInfo and LoGetMechInfo() or nil
    add_probe_numbers(values, "engine", engine, "", 0)
    add_probe_numbers(values, "heli", fm, "", 0)
    add_probe_numbers(values, "mech", mech, "", 0)

    for _, item in ipairs(values) do
        AutoRudderExport.udp:send(string.format(
            "ARP,%.3f,%s,%s,%s,%.8f\n",
            now,
            safe_probe_text(aircraft),
            safe_probe_text(item.source),
            safe_probe_text(item.key),
            item.value))
    end
end

local function export_frame()
    if not ensure_socket() then
        return
    end

    if LoIsOwnshipExportAllowed and not LoIsOwnshipExportAllowed() then
        return
    end

    local now = LoGetModelTime and LoGetModelTime() or 0
    if now - AutoRudderExport.last_time < (1 / AutoRudderExport.rate_hz) then
        return
    end
    AutoRudderExport.last_time = now

    local self_data = LoGetSelfData and LoGetSelfData() or nil
    local aircraft = "NONE"
    if self_data and self_data.Name then
        aircraft = tostring(self_data.Name)
    end

    send_power_probe(now, aircraft)

    local angular = LoGetAngularVelocity and LoGetAngularVelocity() or nil
    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    local angle_of_attack = LoGetAngleOfAttack and LoGetAngleOfAttack() or nil
    local indicated_airspeed = LoGetIndicatedAirSpeed and LoGetIndicatedAirSpeed() or nil
    local true_airspeed = LoGetTrueAirSpeed and LoGetTrueAirSpeed() or nil
    local mach = LoGetMachNumber and LoGetMachNumber() or nil
    local radar_altitude = LoGetAltitudeAboveGroundLevel and LoGetAltitudeAboveGroundLevel() or nil
    local altitude_msl = LoGetAltitudeAboveSeaLevel and LoGetAltitudeAboveSeaLevel() or nil
    local vertical_velocity = LoGetVerticalVelocity and LoGetVerticalVelocity() or nil
    local velocity = LoGetVectorVelocity and LoGetVectorVelocity() or nil
    local wind_velocity = LoGetVectorWindVelocity and LoGetVectorWindVelocity() or nil
    local acceleration = LoGetAccelerationUnits and LoGetAccelerationUnits() or nil
    local mech = LoGetMechInfo and LoGetMechInfo() or nil
    local engine = LoGetEngineInfo and LoGetEngineInfo() or nil
    local pitch, bank, yaw = nil, nil, nil
    if LoGetADIPitchBankYaw then
        pitch, bank, yaw = LoGetADIPitchBankYaw()
    end
    if type(pitch) ~= "number" and self_data and type(self_data.Pitch) == "number" then
        pitch = self_data.Pitch
    end
    if type(bank) ~= "number" and self_data and type(self_data.Bank) == "number" then
        bank = self_data.Bank
    end
    if type(yaw) ~= "number" and self_data and type(self_data.Heading) == "number" then
        yaw = self_data.Heading
    end
    local lat = nested_number(self_data, { "LatLongAlt", "Lat" })
    local lon = nested_number(self_data, { "LatLongAlt", "Long" })
    if type(altitude_msl) ~= "number" then
        altitude_msl = nested_number(self_data, { "LatLongAlt", "Alt" })
    end
    local velocity_x = vector_number(velocity, "x")
    local velocity_y = vector_number(velocity, "y")
    local velocity_z = vector_number(velocity, "z")
    local speed_3d = vector_speed(velocity_x, velocity_y, velocity_z)
    local ground_speed = horizontal_speed_xz(velocity_x, velocity_z)
    local accel_x = vector_number(acceleration, "x")
    local accel_y = vector_number(acceleration, "y")
    local accel_z = vector_number(acceleration, "z")
    local wind_x = vector_number(wind_velocity, "x")
    local wind_y = vector_number(wind_velocity, "y")
    local wind_z = vector_number(wind_velocity, "z")
    local gear_position = find_number_by_keys(mech, { "gear", "Gear" })
    local flaps_position = find_number_by_keys(mech, { "flaps", "Flaps", "flap", "Flap" })
    local rpm_avg = average_pair(engine, "RPM")
    local fuel_avg = average_pair(engine, "FuelConsumption")
    local torque_left_arg = cockpit_argument(982)
    local torque_right_arg = cockpit_argument(983)
    local torque_arg_avg = nil
    if valid_torque_argument(torque_left_arg) and valid_torque_argument(torque_right_arg) then
        torque_arg_avg = (torque_left_arg + torque_right_arg) * 0.5
    elseif valid_torque_argument(torque_left_arg) then
        torque_arg_avg = torque_left_arg
    elseif valid_torque_argument(torque_right_arg) then
        torque_arg_avg = torque_right_arg
    else
        torque_arg_avg = nil
    end
    local torque_avg =
        torque_arg_avg or
        average_first_available(engine, { "Torque", "torque", "TQ", "tq", "EngineTorque", "engineTorque", "engine_torque", "MastTorque", "mast_torque" }) or
        average_first_available(fm, { "Torque", "torque", "TQ", "tq", "EngineTorque", "engineTorque", "engine_torque", "MastTorque", "mast_torque" })
    local tail_rudder_left = nested_number(mech, { "controlsurfaces", "rudder", "left" })
    local tail_rudder_right = nested_number(mech, { "controlsurfaces", "rudder", "right" })
    local yaw_z = angular and safe_number(angular.z) or 0
    local yaw_y = angular and safe_number(angular.y) or 0
    local yaw_accel_z = nested_number(fm, { "angular_acceleration", "z" })
    local slip = LoGetSlipBallPosition and safe_number(LoGetSlipBallPosition()) or 0
    local collective = find_collective()
    local heading = find_heading(self_data)
    local collective_text = ""
    local heading_text = ""
    local aoa_text = ""
    local roll_rate_text = ""
    local ias_text = ""
    local agl_text = ""
    local gear_text = ""
    local flaps_text = ""
    local rpm_text = optional_number_text(rpm_avg)
    local fuel_text = optional_number_text(fuel_avg)
    local torque_text = optional_number_text(torque_avg)
    local torque_left_text = optional_number_text(torque_left_arg)
    local torque_right_text = optional_number_text(torque_right_arg)
    local tail_left_text = optional_number_text(tail_rudder_left)
    local tail_right_text = optional_number_text(tail_rudder_right)
    local yaw_accel_text = optional_number_text(yaw_accel_z)
    local pitch_text = optional_number_text(pitch)
    local bank_text = optional_number_text(bank)
    local yaw_text = optional_number_text(yaw)
    local velocity_x_text = optional_number_text(velocity_x)
    local velocity_y_text = optional_number_text(velocity_y)
    local velocity_z_text = optional_number_text(velocity_z)
    local speed_3d_text = optional_number_text(speed_3d)
    local ground_speed_text = optional_number_text(ground_speed)
    local vertical_velocity_text = optional_number_text(vertical_velocity)
    local true_airspeed_text = optional_number_text(true_airspeed)
    local mach_text = optional_number_text(mach)
    local altitude_msl_text = optional_number_text(altitude_msl)
    local lat_text = optional_number_text(lat)
    local lon_text = optional_number_text(lon)
    local accel_x_text = optional_number_text(accel_x)
    local accel_y_text = optional_number_text(accel_y)
    local accel_z_text = optional_number_text(accel_z)
    local wind_x_text = optional_number_text(wind_x)
    local wind_y_text = optional_number_text(wind_y)
    local wind_z_text = optional_number_text(wind_z)
    if collective ~= nil then
        if collective < 0 then collective = 0 end
        if collective > 1 then collective = 1 end
        collective_text = string.format("%.8f", collective)
    end
    if heading ~= nil then
        heading_text = string.format("%.8f", heading)
    end
    if type(angle_of_attack) == "number" then
        aoa_text = string.format("%.8f", angle_of_attack)
    end
    if angular and type(angular.x) == "number" then
        roll_rate_text = string.format("%.8f", angular.x)
    end
    if type(indicated_airspeed) == "number" then
        ias_text = string.format("%.8f", indicated_airspeed)
    end
    if type(radar_altitude) == "number" then
        agl_text = string.format("%.8f", radar_altitude)
    end
    if type(gear_position) == "number" then
        gear_text = string.format("%.8f", gear_position)
    end
    if type(flaps_position) == "number" then
        flaps_text = string.format("%.8f", flaps_position)
    end

    AutoRudderExport.udp:send(string.format(
        "AR1,%.3f,%s,%.8f,%.8f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%.8f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
        now,
        aircraft,
        yaw_z,
        slip,
        collective_text,
        heading_text,
        aoa_text,
        roll_rate_text,
        ias_text,
        agl_text,
        gear_text,
        flaps_text,
        rpm_text,
        fuel_text,
        tail_left_text,
        tail_right_text,
        yaw_accel_text,
        torque_text,
        torque_left_text,
        torque_right_text,
        yaw_y,
        pitch_text,
        bank_text,
        yaw_text,
        velocity_x_text,
        velocity_y_text,
        velocity_z_text,
        speed_3d_text,
        ground_speed_text,
        vertical_velocity_text,
        true_airspeed_text,
        mach_text,
        altitude_msl_text,
        lat_text,
        lon_text,
        accel_x_text,
        accel_y_text,
        accel_z_text,
        wind_x_text,
        wind_y_text,
        wind_z_text))
end

local previous_after_next_frame = LuaExportAfterNextFrame
LuaExportAfterNextFrame = function()
    if previous_after_next_frame then
        previous_after_next_frame()
    end
    pcall(export_frame)
end
