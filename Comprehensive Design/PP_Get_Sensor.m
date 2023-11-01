function [ S_Sensor ] = PP_Get_Sensor(Robot_State, S_Map, S_Sensor)      

% PP_Get_Sensor라는 함수는 Robot_State, S_Map, S_Sensor를 input으로 받고 LRF를 return 하는 함수이다.
%   Sensor에서 얻은 값을 반환
%   자세한 설명 위치

%% Laser Range Finder (LRF) 센서
MAX_Range = S_Sensor.lrfMaxRange;                  % S_Sensor에 대해서는 line 32 참고
Depth_Resolution = S_Sensor.lrfResolutionDepth;
FoV = S_Sensor.lrfFov;                             % Field Of View (시야)
Angle_Resolution = abs(S_Sensor.lrfResolutionAngle);

LRF = zeros(FoV / Angle_Resolution + 1, 2);         % 굳이 A x 2의 배열 형태로 만든 이유는 line 236에서 2번째 행을 써먹기 위해
LRF(:, 1) = -FoV / 2 : Angle_Resolution : FoV/2;    % -180부터 180까지 1씩 간격을 맞춘 배열

angle_line_W = LRF(:,1) - Robot_State(4);           % Robot_State(4)는 theta를 의미
dx = cosd(angle_line_W);
dy = sind(angle_line_W);

for i = 1 : size(LRF,1)
    for d = 1 : Depth_Resolution : MAX_Range
        x = round(Robot_State(1)+d*dx(i));
        y = round(Robot_State(2)+d*dy(i));
        
        if S_Map.arrMap(x,y) == 0
            LRF(i,2) = d;
            break;
        end
        if d == MAX_Range
            LRF(i,2) = MAX_Range;
        end
    end
end

S_Sensor.lrfData = LRF(:,:);
end