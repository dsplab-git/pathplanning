function [Map, Sensor, State] = PP_Set_Init(mapfile, sizeRobot)
    [pointStart, pointEnd, arrMap] = PP_Load_Map(mapfile);
    [Map, Sensor, State]= PP_Init(pointStart, pointEnd, arrMap, sizeRobot);
end

function [Map, Sensor, State] = PP_Init(pointStart, pointEnd, arrMap, sizeRobot)
% PP_Set_Init이라는 이름의 함수는 PP_Load_Map에서 생성된 pointStart, pointEnd, arrMap
% 그리고 사용자가 정의한 로봇의 크기 sizeRobot 입력 받아
% Map 구조체와 로봇의 상태 행렬 State를 return한다.
% Map은 arrMap, pointStart, pointEnd, coditionArrival, sizeRobot 정보를 포함한다.
% arrMap               환경 [(n x m) binary value]
% pointStart        시작 위치 [x, y, theta]
% pointEnd          도착 위치 [x, y, theta]
% coditionArrival   도착으로 판단하는 조건으로 로봇 크기를 기준으로 함
% sizeRobot         로봇의 크기
%
% Sensor는 LRF센서 lrfMaxRange,lrfResolutionDepth,lrfFov,lrfResolutionAngle
% lrfMaxRange           최대 측정 거리
% lrfResolutionDepth    거리에 대한 분해능
% lrfFov                최대 측정 각도 범위
% lrfResolutionAngle    각도에 대한 분해능
%
% State는 로봇의 상태를 x, y, v, theta 이 4가지로 정의한다.

%% Map Infomation
Map = struct();                     % 구조체 선언
Map.arrMap = arrMap;                % arrMap 멤버
Map.pointStart = pointStart;        % 시작 정보를 저장하는 point 멤버
Map.pointEnd = pointEnd;            % 도착 정보를 저장하는 point 멤버
Map.coditionArrival = sizeRobot;    % 도착 조건
Map.sizeRobot = sizeRobot;          % 로봇 사이즈

%% Set Sensor state
Sensor = struct();                  % 구조체 선언
% LRF (Laser Range Finder)
Sensor.lrfMaxRange = 100;           % 최대거리 멤버
Sensor.lrfResolutionDepth = 1;      % 깊이 resolution 멤버
Sensor.lrfFov = 270;                % 각도 범위 멤버
Sensor.lrfResolutionAngle = 1;      % 각도 resolution 멤버
Sensor.lrfData = zeros(Sensor.lrfFov/Sensor.lrfResolutionAngle,2); % 센서의 거리값 저장

%% Robot Infomation
State = [0 0 0 0]';  
State(1,1) = pointStart(1);
State(2,1) = pointStart(2);
State(3,1) = 0;
State(4,1) = pointStart(3);

end

function [ pointStart, pointEnd, map ] = PP_Load_Map( mapfile, flag_show )
% PP_Load_Map이라는 이름의 함수는 mapfile을 인자로 전달받고 start_point, end_point 그리고 map을 return한다.
% Path Planning(PP)을 위한 환경 정보를 불러옴
% png 형태로 작성된 이미지에서 free space(자유공간)와 obstacle(장애물) 정보를 읽고
% 붉은색(시작), 파란색(도착), 초록색(방향) 정보를 읽어 시작, 도착 정보를 생성

% start_point 시작 위치 [x, y, theta]
% end_point   도착 위치 [x, y, theta]
% map         실험 환경 [(n x m) binary value]

%% Set Default Value
if nargin < 2
    flag_show = 1;
end

I = imread(mapfile);     % imread(filename)는 A x B x 3 array로 이미지를 읽어온다.

R = im2bw(I(:,:,1),0.5); % im2bw는 threshold 값을 기반으로 이미지를 바이너리 이미지로 convert해주는 함수
G = im2bw(I(:,:,2),0.5); % BW = im2bw(RGB,level); converts the truecolor image RGB to a binary image.
B = im2bw(I(:,:,3),0.5);

%% Map Build
map = xor(xor(R,G),B);   % 배열 R과 G의 논리 배타적 OR을 구한뒤 1혹은 0으로 반환되는 결과와 배열 B의 배타적 OR을 구한다.

%% Find Start and End Point
for i = 1 : size(I,1)                       % x 값들
    for j = 1 : size(I,2)                   % y 값들
        % Start Point
        if(R(i,j) & ~G(i,j) & ~B(i,j))      % RED가 시작
            pointStart = [i;j;0];
        end
        % End Point
        if(~R(i,j) & ~G(i,j) & B(i,j))      % BLUE가 도착
            pointEnd = [i;j;0];
        end
    end
end

%% Find Direction
% Start
Flag_find = 0;
for i = -2 : 2
    for j = -2 : 2
        x = pointStart(1)+i;
        y = pointStart(2)+j;
        if(~R(x,y) & G(x,y) & ~B(x,y))                  % G는 방향정보
            dx = x - pointStart(1);
            dy = y - pointStart(2);
            pointStart(3) = atan2d(dy,dx);

            Flag_find = Flag_find+1;
        end
    end
end
if(Flag_find ~= 1)
    % 녹색 점 문제
    Flag_find
    return;
end
% End
Flag_find = 0;
for i = -2 : 2
    for j = -2 : 2
        x = pointEnd(1)+i;
        y = pointEnd(2)+j;
        if(~R(x,y) & G(x,y) & ~B(x,y))
            dx = x - pointEnd(1);
            dy = y - pointEnd(2);
            pointEnd(3) = atan2d(dy,dx);

            Flag_find = Flag_find+1;
        end
    end
end
if(Flag_find ~= 1)
    % 녹색 점 문제
    Flag_find
    return;
end

% Show Result
if flag_show
    L = 3;
    figure(1);           % figure(n) finds a figure in which the Number property is equal to n, and makes it the current figure.
    imshow(map)
    hold on             % 새로운 plot을 추가하더라도 현 plot을 유지하기 위한 command
    scatter(pointStart(2),pointStart(1),(2*L)^2);
    plot([pointStart(2), pointStart(2)+L*sind(pointStart(3))],[pointStart(1), pointStart(1)+L*cosd(pointStart(3))])   % cosd, sind는 단위가 degree이고 싶을때 사용
    scatter(pointEnd(2),pointEnd(1),(2*L)^2,'xr');            % 산점도 plot 'd'는 다이아몬드, 'xr'은 cross이고 red를 의미
    hold off            % 새로운 plot을 추가할 때 현 plot을 유지하지 않겠다는 command
end
end