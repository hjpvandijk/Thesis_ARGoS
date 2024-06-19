myAgent = nil
function init()
   package.cpath = package.cpath .. ";build/agent_implementation/lib?.so"
   local agent = require("agent_implementation");
   myAgent = agent.agent(robot.id, agent.coordinate(0,0));

end

function step()
   log('agent id ' .. myAgent:getId())
   -- Get the position of the agent
  --local x = myAgent:getPosition():getX()
  --local y = myAgent:getPosition():getY()
  local position = myAgent:getPosition()
    local x = position:getX()
    local y = position:getY()
    --print coordinates
    log('x: ' .. x .. ' y: ' .. y)

   -- Get the parameters of the agent
   --log('alpha: ' .. robot.params.alpha)
   --log('delta: ' .. robot.params.delta)
   --log('velocity: ' .. robot.params.velocity)

   -- Send a radio message
      robot.simple_radios.wifi.send({'ping'});

    -- Receive radio messages and respond
   for index, message in ipairs(robot.simple_radios.wifi.recv) do
      if message[1] == 'ping' then
         robot.simple_radios.wifi.send({'pong'})
         --log('ping')
      elseif message[1] == 'pong' then
         robot.simple_radios.wifi.send({'ping'})
         --log('pong')
      end
   end

    -- Get the proximity sensor readings
   for i, sensor in ipairs(robot.rangefinders) do
       log('rangefinder ' .. i .. ':')
       log('  proximity = ' .. sensor.proximity)
       log('  illuminance = ' .. sensor.illuminance)
       --log('  transform:')
       --log('    anchor = ' .. sensor.transform.anchor)
       --log('    position = ' .. sensor.transform.position)
       --log('    orientation = ' .. sensor.transform.orientation)
     end

     -- Move the robot randomly
       robot.differential_drive.set_linear_velocity(math.random()*0.1, math.random()*0.1)
end

function reset()
end

function destroy()
end

