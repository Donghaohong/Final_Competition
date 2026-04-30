function[dataStore] = backupBump(Robot,maxTime)

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 50000;
end

try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;

dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);

disp('*** NEW motionControl is running ***')
disp(fieldnames(dataStore))

noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0,0);
tic

bumped = false;

state = 0;

backstart = [0 0];

turnstart = 0;

while toc < maxTime

    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    if state == 0
        bumped = false;
        if ~isempty(dataStore.bump)
            BumpRight = dataStore.bump(end, 2);
            BumpLeft  = dataStore.bump(end, 3);
            BumpFront = dataStore.bump(end, 7);
            bumped = BumpRight || BumpLeft || BumpFront;
        end
    end

    if bumped && state == 0
        state = 1; 
        backstart = dataStore.truthPose(end, 2:3);
        bumped = false;
        SetFwdVelAngVelCreate(Robot, 0,0);
    end

    wheel2Center = 0.13;
    maxV = 0.1;

    if state == 1
        curxy = dataStore.truthPose(end, 2:3);
        distmove = norm(curxy - backstart);

        if distmove >= 0.25
            SetFwdVelAngVelCreate(Robot, 0,0);
            pause(0.1); 
            turnstart = dataStore.truthPose(end,4);
            state = 2;
        else
            cmdV = -0.2;
            cmdW = 0;

            [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);

            if noRobotCount >= 3
                SetFwdVelAngVelCreate(Robot, 0,0);
            else
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
            end
        
        end

    elseif state == 2
        curtheta = dataStore.truthPose(end, 4);

        dtheta = atan2(sin(curtheta - turnstart), cos(curtheta - turnstart));

        if abs(dtheta) >= deg2rad(30)
            SetFwdVelAngVelCreate(Robot, 0, 0);
            state = 0;
        else
            cmdV = 0;
            cmdW = -0.4; 

            [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);

            if noRobotCount >= 3
                SetFwdVelAngVelCreate(Robot, 0,0);
            else
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
            end 

        end

    else

        cmdV = 0.2;
        cmdW = 0;
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0,0);
        else
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
        end 

    end

end

SetFwdVelAngVelCreate(Robot, 0,0 );
