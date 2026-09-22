-- Hardware regression check: use a display initially at a different refresh
-- rate from a CFR video. This changes the display mode and quits after testing.
-- Run with --display-rate-match=yes --display-rate-match-delay=3 and this script.
-- Add --script-opts=display_pause_test-manual_pause=yes to check user pause.
local mp = require 'mp'
local options = require 'mp.options'
local opts = {manual_pause = false}
options.read_options(opts, 'display_pause_test')

local started, position, resumed, finished
local delay, ticker

local function finish(ok, message)
    if finished then return end
    finished = true
    if ticker then ticker:kill() end
    if ok then mp.msg.info('PASS: ' .. message)
    else mp.msg.error('FAIL: ' .. message) end
    mp.commandv('quit', ok and 0 or 1)
end

mp.observe_property('core-idle', 'bool', function(_, idle)
    if finished then return end
    local pts = mp.get_property_number('time-pos', 0)
    if not started and idle and pts > 1 and
       not mp.get_property_bool('pause') and
       not mp.get_property_bool('paused-for-cache') then
        started, position = mp.get_time(), pts
        delay = mp.get_property_number('display-rate-match-delay', 0)
        if delay < 1 then
            finish(false, 'this check requires a settling delay of at least 1s')
            return
        end
        mp.msg.info(string.format('Settling hold detected at %.3fs', pts))
        if opts.manual_pause then mp.set_property_bool('pause', true) end
    elseif started and not idle and not resumed then
        if mp.get_time() - started < delay - 0.2 then
            finish(false, 'playback resumed before the settling delay elapsed')
        else
            resumed = true
            mp.msg.info('Playback resumed after settling')
        end
    end
end)

ticker = mp.add_periodic_timer(0.05, function()
    if not started or finished then return end
    local elapsed = mp.get_time() - started
    local pts = mp.get_property_number('time-pos', position)
    if not resumed and math.abs(pts - position) > 0.1 then
        finish(false, 'media time advanced while playback was held')
        return
    end
    if opts.manual_pause and elapsed > delay + 0.5 then
        if not mp.get_property_bool('pause') or
           not mp.get_property_bool('core-idle') then
            finish(false, 'automatic resume overwrote the user pause')
            return
        end
        opts.manual_pause = false
        mp.msg.info('User pause preserved after settling; releasing it')
        mp.set_property_bool('pause', false)
    end
    if resumed and pts > position + 0.5 then
        finish(true, 'media time held during refresh settling, then resumed')
    end
end)

mp.add_timeout(25, function()
    finish(false, 'no completed refresh pause/resume; check initial display mode')
end)
