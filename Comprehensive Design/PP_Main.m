clc; clear; close all; % 프로그램 실행과 동시에 기존에 열려있던 창과 명령 창을 다 닫기

mapfile = 'map1.png'; % 지도 정보

% Set variable
Robot_State = [0 0 0 0]';  % 여기서는 로봇의 상태를 x, y, v, theta 이 4가지로 표현

Control_Input = [];
Control_mode = 2; % [mode 1] : d_v d_theta | [mode 2] : d_x d_y | [mode 3] : vl  vr

% Initialize
[start_point,end_point,map] = PP_Load_Map(mapfile); % line 124에 정의된 함수
clear mapfile

%% Map & Robot Infomation
S_Map = struct();                       % 구조체 선언
S_Map.map = map;                        % map 멤버 --> 아래에서 map = xor(xor(R,G),B)로 정의함
S_Map.point_start = start_point;        % start point 멤버
S_Map.point_end = end_point;            % end point 멤버
S_Map.arrive_codition = 10;             % 도착 조건
S_Map.robot_size = 5;                   % 로봇 사이즈

%% Set Robot State
Robot_State(1,1) = S_Map.point_start(1);
Robot_State(2,1) = S_Map.point_start(2);
Robot_State(3,1) = 0;
Robot_State(4,1) = S_Map.point_start(3);

%% Save Init Point
Robot_State_Save = Robot_State;
Robot_State_Array = Robot_State_Save;
Robot_Input_Array = [0; 0];

%% Set Sensor state
S_Sensor = struct();                    % 구조체 선언
% LRF (Laser Range Finder)
S_Sensor.LRF_MaxRange = 100;            % 최대거리 멤버
S_Sensor.LRF_Depth_Resolution = 1;      % 깊이 resolution 멤버
S_Sensor.LRF_FOV = 270;                 % 각도 범위 멤버
S_Sensor.LRF_Angle_Resolution = 1;      % 각도 resolution 멤버

% Q2 고정된 장애물을 회피하기 위한 중간 경로를 설정한다 
% Input = 장애물 정보 ( map ) ,  출발지와 목적지 ( start_point,end_point )
% Output = Mid_Point
%%%%%%%%%%%%%%%%%%% 코드에서 사용하는 초기 변수 선언부 %%%%%%%%%%%%%%%%%%%%%         --> Hint?
% Q1의 해결을 위한 코드
Mid_Point = [281, 73];         
         
Mid_State = 1;
Ref_Speed = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



while(norm(Robot_State(1:2) - S_Map.point_end(1:2)) > S_Map.arrive_codition)   % Real; Robot이 arrive condition을 만족할 때까지 -----> 이 부분이 내가 핵심적으로 건드려야 할 부분임
% for Loop_Num = 1 : 2                                                         % Test
    %% Sensor Data
     LRF = PP_Sensor(Robot_State, S_Map, S_Sensor);

    %% Path Planning
    %%%%%%%%%%%%%%%%%%%%%%%%%%%코드 작성 부%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Info
    % 현재 위치 (Robot_State(1), Robot_State(2))
    % 목표 지점 (end_point(1), end_point(2))
    % 장애물 정보는 map에 저장되어 있음
    % Sensor 정보는 LRF 변수에 저장되어 있음
    
    % 중간 지점을 목적지로하여 로봇이 이동함
    Goal_x = Mid_Point(Mid_State,1);
    Goal_y = Mid_Point(Mid_State,2);

    SensorData = LRF;

    A_control = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);
    
    if(Robot_State(3) > Ref_Speed)
        V_control = 0;
    else 
        V_control = 1;
    end
    
    
    % Q1 장애물이 있는 경우 LRF에 값이 입력됨 이를 이용하여 회피하는 알고리즘
    % Input = 중간 목적지 (Goal_x, Goal_y), 주변 장애물 정보 (LRF)
    % Output = 제어 입력 (A_control, V_contorl)
    
    
    
    
    % 중간 목적지에 도달한 경우 다음 지점을 향해서 이동함
    if(norm([Goal_y - Robot_State(2),Goal_x - Robot_State(1)]) < Ref_Speed)
        Mid_State = Mid_State+1;
    end
    
    % 목적지 도착시 종료
    if Mid_State > size(Mid_Point,1)
        break;
    end
    
    % PP output  
    Control_Input = [V_control;A_control];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    %% Robot Control
    Robot_State = PP_Robot_Control(Robot_State, Control_Input, Control_mode)';
    
    %% Data Save
    Robot_State_Save(1) = Robot_State(1);
    Robot_State_Save(2) = Robot_State(2);
    Robot_State_Save(3) = Robot_State(3);
    Robot_State_Save(4) = Robot_State(4);
    
    Robot_State_Array = [Robot_State_Array, Robot_State_Save];
    Robot_Input_Array = [Robot_Input_Array, Control_Input];
    
    %% Collision Check
    Flag_Collision = PP_Collision_Check( Robot_State_Array, S_Map);
    
    if(Flag_Collision)
        Robot_State_Array = Robot_State_Array(:,1:end-1);
        break;
    end
    
    %% Show Position and Sensor Data
    PP_Show_Path(Robot_State_Array, S_Map, 1);
    
    figure(2);
    polarplot((LRF(:,1) + Robot_State(4))*pi/180,LRF(:,2)')                             % LRF 원형 그래프를 보여주기 위해!
    drawnow                                                                             % drawnow가 호출되는 시점마다 그래프가 그려진다.
    
    
end

%% Show Result
PP_Show_Path(Robot_State_Array, S_Map, 10);
disp(['Total Step is : ', num2str(size(Robot_State_Array,2))]);

%% Functions
function [ start_point, end_point, map ] = PP_Load_Map( mapfile ) 
% PP_Load_Map이라는 이름의 함수는 mapfile을 인자로 전달받고 start_point, end_point 그리고 map을 return한다.  
% Path Planning(PP)을 위한 환경 정보를 불러옴
% png 형태로 작성된 이미지에서 free space(자유공간)와 obstacle(장애물) 정보를 읽고
% 붉은색(시작), 파란색(도착), 초록색(방향) 정보를 읽어 시작, 도착 정보를 생성

% start_point 시작 위치 [x, y, theta]
% end_point   도착 위치 [x, y, theta]
% map         실험 환경 [(n x m) binary value]
    
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
                start_point = [i;j;0];
            end
            % End Point
            if(~R(i,j) & ~G(i,j) & B(i,j))      % BLUE가 도착
                end_point = [i;j;0];
            end
        end
    end

    %% Find Direction
    % Start
    Flag_find = 0;
    for i = -2 : 2
        for j = -2 : 2
            x = start_point(1)+i;
            y = start_point(2)+j;
            if(~R(x,y) & G(x,y) & ~B(x,y))                  % G는 방향정보
                dx = x - start_point(1);
                dy = y - start_point(2);
                start_point(3) = atan2d(dy,dx);

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
            x = end_point(1)+i;
            y = end_point(2)+j;
            if(~R(x,y) & G(x,y) & ~B(x,y))
                dx = x - end_point(1);
                dy = y - end_point(2);
                end_point(3) = atan2d(dy,dx);

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
    figure(1);           % figure(n) finds a figure in which the Number property is equal to n, and makes it the current figure. 
                        % If no figure exists with that property value, MATLAB creates a new figure and sets its Number property to n.
    imshow(map')        
    hold on             % 새로운 plot을 추가하더라도 현 plot을 유지하기 위한 command
    L = 3;
    scatter(start_point(2),start_point(1),(2*L)^2);
    plot([start_point(2), start_point(2)+L*sind(start_point(3))],[start_point(1), start_point(1)+L*cosd(start_point(3))])   % cosd, sind는 단위가 degree이고 싶을때 사용
    
    scatter(end_point(2),end_point(1),(2*L)^2,'xr');            % 산점도 plot 'd'는 다이아몬드, 'xr'은 cross이고 red를 의미
    
    hold off            % 새로운 plot을 추가할 때 현 plot을 유지하지 않겠다는 command

end
function [ LRF ] = PP_Sensor(Robot_State, S_Map, S_Sensor)      % PP_Sensor라는 함수는 Robot_State, S_Map, S_Sensor를 input으로 받고 LRF를 return 하는 함수이다.
%   Sensor에서 얻은 값을 반환
%   자세한 설명 위치

%% Laser Range Finder (LRF) 센서
MAX_Range = S_Sensor.LRF_MaxRange;                  % S_Sensor에 대해서는 line 32 참고
Depth_Resolution = S_Sensor.LRF_Depth_Resolution;
FoV = S_Sensor.LRF_FOV;                             % Field Of View (시야)
Angle_Resolution = S_Sensor.LRF_Angle_Resolution;

LRF = zeros(FoV / Angle_Resolution + 1, 2);         % 굳이 A x 2의 배열 형태로 만든 이유는 line 236에서 2번째 행을 써먹기 위해
LRF(:, 1) = -FoV / 2 : Angle_Resolution : FoV/2;    % -180부터 180까지 1씩 간격을 맞춘 배열

angle_line_W = Robot_State(4) - LRF(:,1);           % Robot_State(4)는 theta를 의미
dx = cosd(angle_line_W);
dy = sind(angle_line_W);

for i = 1 : size(LRF,1)
    for d = 1 : Depth_Resolution : MAX_Range
        x = round(Robot_State(1)+d*dx(i));
        y = round(Robot_State(2)+d*dy(i));
        
        if S_Map.map(x,y) == 0
            LRF(i,2) = d;
            break;
        end
        if d == MAX_Range
            LRF(i,2) = MAX_Range;
        end
    end
end

end
function [ Robot_State_output ] = PP_Robot_Control( Robot_State, Control_Input, Mode )      
% PP_Robot_Control이라는 함수는 Robot_State, Control_Input, Mode 인자를 받고 Robot_State_output을 return 함

% Mode 1 : x,y,dx,dy
% robot_state = [x,y,v,theta]

% Mode 2 : x,y,v,theta
% robot_state = [x,y,v,theta]

% Mode 3 : x,y,vl,vr
% robot_state = [x,y,v,theta]

% Robot Size
L = 10;

switch(Mode)        % switch 문
    case 1
        Robot_State_output(3) = sqrt(Control_Input'*Control_Input);     % Control_Input은 line 86에 정의되어있음; Control_Input = [V_control;A_control];
        Robot_State_output(4) = atan2d(Control_Input(1),Control_Input(2));
        Robot_State_output(1) = Robot_State(1) + Control_Input(1);
        Robot_State_output(2) = Robot_State(2) + Control_Input(2);
    case 2
        Robot_State_output(3) = Robot_State(3) + Control_Input(1);
        Robot_State_output(4) = Robot_State(4) + Control_Input(2);
        Robot_State_output(1) = Robot_State(1) + Robot_State(3)*cosd(Robot_State(4));
        Robot_State_output(2) = Robot_State(2) + Robot_State(3)*sind(Robot_State(4));
    case 3
        v = (Control_Input(1) + Control_Input(2))/2;
        d_theta = (Control_Input(1) - Control_Input(2))/L;
        Robot_State_output(3) = Robot_State(3) + v;
        Robot_State_output(4) = Robot_State(4) + d_theta;
        Robot_State_output(1) = Robot_State(1) + Robot_State(3)*cosd(Robot_State(4));
        Robot_State_output(2) = Robot_State(2) + Robot_State(3)*sind(Robot_State(4));
end

end
function [ Flag_Collision ] = PP_Collision_Check( Robot_State_Array, S_Map)     % PP_Collision_Check라는 함수는 Robot_State_Array와 S_Map을 인자로 받아 Flag_Collision을 return 함
%% Collision Check
Flag_Collision = 1;                                         % Flag_Collision은 0, 1, 2 중 하나의 값을 가질 수 있음
Robot_State = Robot_State_Array(:,end);
map = S_Map.map;
End_Point = S_Map.point_end;
Arrive_Condition = S_Map.arrive_codition;

% map 범위를 벗어나는지 확인
if(Robot_State(1) < 1 || Robot_State(1) > size(map,1))      % Robot_State(1)은 x를 의미
    return;
end
if(Robot_State(2) < 1 || Robot_State(2) > size(map,2))      % Robot_State(2)는 y를 의미
    return;
end
% 이전 값과 현재 값 사이에 장애물이 있는지 확인
Post_Robot_State = Robot_State;
Pre_Robot_State = Robot_State_Array(:,end-1);

Post_Robot_State = round(Robot_State(1:2));
Pre_Robot_State = round(Pre_Robot_State(1:2));

% 사각형 테스트
for i = min(Post_Robot_State(1), Pre_Robot_State(1)) : max(Post_Robot_State(1), Pre_Robot_State(1))
    for j = min(Post_Robot_State(2), Pre_Robot_State(2)) : max(Post_Robot_State(2), Pre_Robot_State(2))
        if(~map(i,j))
            return;
        end
    end
end

Flag_Collision = 0;

% 도착지점
if norm(Robot_State(1:2) - End_Point(1:2)) < Arrive_Condition
    Flag_Collision = 2;                                             % 2는 도착을 의미
end
end
function PP_Show_Path( Robot_State_Array, S_Map, N_figure )
% PP_Show_Path라는 함수는 Robot_State_Array, S_Map, N_figure를 인자로 받고 return 하지않는 void 함수인듯

    %% Load Map info
    map  = S_Map.map;
    start_point = S_Map.point_start;
    end_point = S_Map.point_end;
    
    %% Robot Info
    robot_size = S_Map.robot_size;
    
    %% Show Path
    Robot_State = Robot_State_Array(:,end);

    figure(N_figure)
    L = 3;
    imshow(map')
    hold on
    % start
    scatter(start_point(1),start_point(2),(2*L)^2,'or');
    % path
    plot(Robot_State_Array(1,1:end),Robot_State_Array(2,1:end),'k.-')
    % present
    plot(robot_size * cos(0:pi/10:2*pi) + Robot_State(1),robot_size * sin(0:pi/10:2*pi) + Robot_State(2),'b');
    plot([Robot_State(1),1.5*robot_size*cosd(Robot_State(4))+Robot_State(1)],[Robot_State(2),1.5*robot_size*sind(Robot_State(4))+Robot_State(2)],'r');
    scatter(Robot_State(1),Robot_State(2),(2*L)^2,'b');
    % end
    scatter(end_point(1),end_point(2),(2*L)^2,'xr');
    plot(S_Map.arrive_codition * cos(0:pi/10:2*pi) + end_point(1),S_Map.arrive_codition * sin(0:pi/10:2*pi) + end_point(2),'r')
    hold off
   
end