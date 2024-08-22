myAgent = nil
rangerfinder = nil
function init()
   package.cpath = package.cpath .. ";build/agent_implementation/lib?.so"
   local agent = require("agent_implementation");
   local agentCoordinate = agent.Coordinate;
   myAgent = agent.agent(robot.id);
   rangefinder = robot.rangefinders[1]

end

function step()
   log('agent id ' .. myAgent:getId())
   local pos = robot.positioning.position
   local x = pos.GetX
   log('x: ' .. x)
   -- Get the position of the agent
  --local x = myAgent:getPosition():getX()
  --local y = myAgent:getPosition():getY()
  --local position = myAgent:getPosition()
  --  local x = position.x
  --  local y = position.y
  --  --print coordinates
  --  log('x: ' .. x .. ' y: ' .. y)

   --log('position: ' .. robot.positioning.position:GetX() .. ', ' .. robot.positioning.position:GetY() .. ', ' .. robot.positioning.position:GetZ())
   --log('positoin type: ' .. swig_type(robot.positioning.position))
   -- log('position : ' .. robot.positioning.position)

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
   --for i, sensor in ipairs(robot.rangefinders) do
   --    log('rangefinder ' .. i .. ':')
   --    log('  proximity = ' .. sensor.proximity)
   --    --log('  illuminance = ' .. sensor.illuminance)
   --    --log('  transform:')
   --    --log('    anchor = ' .. sensor.transform.anchor)
   --    --log('position type ' .. swig_type(sensor.transform.position))
   --    ----log('    position = ' .. sensor.transform.position:GetX() .. ', ' .. sensor.transform.position:GetY() .. ', ' .. sensor.transform.position:GetZ())
   --    --log('orientation type ' .. swig_type(sensor.transform.orientation)
   --    ----log('    orientation = ' .. sensor.transform.orientation)
   --  end

    myAgent:setLastRangeReading(rangefinder.proximity)
    log('agent proximity: ' .. myAgent.lastRangeReading)


     -- Move the robot randomly
       robot.differential_drive.set_linear_velocity(0.1, 0.1)
end

function reset()
end

function destroy()
end

