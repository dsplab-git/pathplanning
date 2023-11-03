clc; clear; close all; % 프로그램 실행과 동시에 기존에 열려있던 창과 명령 창을 다 닫기

% Set variable

Control_Input = [];
Control_mode = 2; % [mode 1] : d_v d_theta | [mode 2] : d_x d_y | [mode 3] : vl  vr

Robot_Size = 5;
MAX_SIMULATION = 10000;
%% Initialize
% Set Map & Sensor & Robot State
[S_Map, S_Sensor, Robot_State]= PP_Set_Init('map1.png', Robot_Size);

%% Record Save Array
Robot_State_Save = Robot_State;
Robot_State_Array = zeros(length(Robot_State_Save),MAX_SIMULATION);
Robot_State_Array_index = 0;

Robot_Input_Array = zeros(2,MAX_SIMULATION);
Robot_State_Array_index = Robot_State_Array_index + 1;
Robot_State_Array(:,Robot_State_Array_index) = Robot_State_Save;

% Q2 고정된 장애물을 회피하기 위한 중간 경로를 설정한다
% Input = 장애물 정보 ( map ) ,  출발지와 목적지 ( start_point,end_point )
% Output = Mid_Point
%%%%%%%%%%%%%%%%%%% 코드에서 사용하는 초기 변수 선언부 %%%%%%%%%%%%%%%%%%%%%         --> Hint?
% Q1의 해결을 위한 코드
Mid_Point = [S_Map.pointEnd(1), S_Map.pointEnd(2)];

Mid_State = 1;
Ref_Speed = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while(norm(Robot_State(1:2) - S_Map.pointEnd(1:2)) > S_Map.coditionArrival)   % Real; Robot이 coditionArrival을 만족할 때까지 -----> 이 부분이 내가 핵심적으로 건드려야 할 부분임
    %% Update
    [S_Sensor] = PP_Get_Sensor(Robot_State, S_Map, S_Sensor);
    LRF = S_Sensor.lrfData;

    %% Path Planning
    %%%%%%%%%%%%%%%%%%%%%%%%%%%코드 작성 부%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Info
    % 현재 위치 (Robot_State(1), Robot_State(2))
    % 목표 지점 (pointEnd(1), pointEnd(2))
    % 장애물 정보는 구조체(S_Map)의 arrMap에 저장되어 있음
    % Sensor 정보는 LRF 변수에 저장되어 있음

    % 중간 지점을 목적지로하여 로봇이 이동함
    Goal_x = Mid_Point(Mid_State,1);
    Goal_y = Mid_Point(Mid_State,2);

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
    Robot_State_Save = Robot_State;
    Robot_State_Array_index = Robot_State_Array_index + 1;
    Robot_State_Array(:,Robot_State_Array_index) = Robot_State_Save;
    Robot_Input_Array(:,Robot_State_Array_index) = Control_Input';

    %% Collision Check
    Flag_Collision = PP_Collision_Check( Robot_State_Array(:,1:Robot_State_Array_index), S_Map);

    if(Flag_Collision)
        Robot_State_Array = Robot_State_Array(:,1:Robot_State_Array_index-1);
        break;
    end

    %% Show Position and Sensor Data
    PP_Show_Path(Robot_State_Array(:,1:Robot_State_Array_index), S_Map, S_Sensor, 2);

end

%% Show Result
PP_Show_Path(Robot_State_Array, S_Map, S_Sensor, 10);
disp(['Total Step is : ', num2str(size(Robot_State_Array,2))]);


