%%%%%%%%%%%%%%%%%%%%%%%% Local Path Planning %%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% 현재 위치 (Robot_State(1), Robot_State(2))
% 목표 지점 (pointEnd(1), pointEnd(2))
% 장애물 정보는 구조체(S_Map)의 arrMap에 저장되어 있음
% Sensor 정보는 LRF 변수에 저장되어 있음

%% Initial Value
if(initial_flag == 0)
    detect = 0;
    target_angle = [];
    escape_flag = 0;
    count = 0;
    count_target = 0;
end

%% 장애물을 탐지하면 detect를 1로 설정하여 flag 세운다.
detect = 0;
for i = 1:size(LRF,1)
     if LRF(i,2) < 20
         disp("detected obstacle")
         detect = 1;
         break;
     end
end

if ((escape_flag == 1) && (count_target < 5) && (detect == 1))% 아직 장애물을 탈출 못한 경우 탈출할 수 있도록 한다.
         detect = 0;
end

if ((escape_flag == 1) && (count_target >= 5) && (detect == 1))% 탈출한 뒤 다시 장애물을 만난 경우.
         detect = 1;
         escape_flag = 0;
         count_target = 0;
end

obstacle_point_x = [];
obstacle_point_y = [];

if(detect == 1)
    %%%%%%%%%%%%%%%% 자동차 조향 제어 %%%%%%%%%%%%%%%%%%%
    % 센서가 주는 장애물 point까지의 거리와 각도 정보를 이용해 장애물의 가장자리 좌표를 구한다.
    for i = 1:size(LRF,1)
        if (LRF(i,2) < 15)
            obstacle_point_x = [obstacle_point_x, Robot_State(1) + LRF(i,2) * cosd(LRF(i,1) - Robot_State(4))];
            obstacle_point_y = [obstacle_point_y, Robot_State(2) + LRF(i,2) * sind(LRF(i,1) - Robot_State(4))];
        end
    end

    % 장애물이 탐지된 경우 자동차가 나아가야할 장애물 가장자리의 point를 정한다.
    if isempty(obstacle_point_x) == 0
        sum_x = 0;
        sum_y = 0;

        for i = 1:size(obstacle_point_x, 2)
            sum_x = sum_x + obstacle_point_x(1);
            sum_y = sum_y + obstacle_point_y(1);
        end

        destination_x = round(sum_x / size(obstacle_point_x, 2));
        destination_y = round(sum_y / size(obstacle_point_y, 2));

        Mid_Point = [Mid_Point; destination_x, destination_y];
    end

    x = Mid_Point(size(Mid_Point,1), 1);
    y = Mid_Point(size(Mid_Point,1), 2);


    % 위에서 구한 point로 자동차가 목표로 하는 waypoint를 구한다.
    if((S_Map.arrMap(x,y+1) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x-1,y) == 0) && (S_Map.arrMap(x,y-1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1);
        Goal_y = Mid_Point(size(Mid_Point,1), 2) + 7 ;
    elseif((S_Map.arrMap(x,y+1) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x-1,y) > 0) && (S_Map.arrMap(x,y-1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) - 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) + 7;
    elseif((S_Map.arrMap(x,y+1) > 0) && (S_Map.arrMap(x+1,y) > 0) && (S_Map.arrMap(x-1,y) == 0) && (S_Map.arrMap(x,y-1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) + 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) + 7;

    elseif((S_Map.arrMap(x-1,y) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x,y-1) == 0) && (S_Map.arrMap(x,y+1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) - 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2);
    elseif((S_Map.arrMap(x-1,y) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x,y-1) > 0) && (S_Map.arrMap(x,y+1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) - 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) - 7;
    elseif((S_Map.arrMap(x-1,y) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x,y-1) == 0) && (S_Map.arrMap(x,y+1) > 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) - 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) + 7;

    elseif((S_Map.arrMap(x,y-1) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x,y+1) == 0) && (S_Map.arrMap(x-1,y) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1);
        Goal_y = Mid_Point(size(Mid_Point,1), 2) - 7;
    elseif((S_Map.arrMap(x,y-1) > 0) && (S_Map.arrMap(x+1,y) > 0) && (S_Map.arrMap(x,y+1) == 0) && (S_Map.arrMap(x-1,y) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) + 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) - 7;
    elseif((S_Map.arrMap(x,y-1) > 0) && (S_Map.arrMap(x+1,y) == 0) && (S_Map.arrMap(x,y+1) == 0) && (S_Map.arrMap(x-1,y) > 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) - 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) - 7;

    elseif((S_Map.arrMap(x+1,y) > 0) && (S_Map.arrMap(x-1,y) == 0) && (S_Map.arrMap(x,y-1) == 0) && (S_Map.arrMap(x,y+1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) + 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2);
    elseif((S_Map.arrMap(x+1,y) > 0) && (S_Map.arrMap(x-1,y) == 0) && (S_Map.arrMap(x,y-1) == 0) && (S_Map.arrMap(x,y+1) > 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) + 5;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) + 5;
    elseif((S_Map.arrMap(x+1,y) > 0) && (S_Map.arrMap(x-1,y) == 0) && (S_Map.arrMap(x,y-1) > 0) && (S_Map.arrMap(x,y+1) == 0))
        Goal_x = Mid_Point(size(Mid_Point,1), 1) + 7;
        Goal_y = Mid_Point(size(Mid_Point,1), 2) - 7;

    else
        Goal_x = Mid_Point(size(Mid_Point,1), 1);
        Goal_y = Mid_Point(size(Mid_Point,1), 2);
    end


    % Waypoint까지의 각도를 활용하여 steering angle을 구한다.
    guide_angle = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);

    if isempty(obstacle_point_x)
        Goal_x = Mid_Point(size(Mid_Point,1), 1);
        Goal_y = Mid_Point(size(Mid_Point,1), 2);
        guide_angle = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);
    end


    % 조향각의 절대값이 180을 넘을때 갑자기 부호가 반전되게 된다.(180 -> 185가 될 때 -175로 처리한다)
    % 조향각의 절대값이 180을 넘을 경우 음수면 360 더해주고 양수면 360 빼준다.
    if (abs(guide_angle) > 180)
        if (guide_angle < 0)
            guide_angle = guide_angle + 360;
        else
            guide_angle = guide_angle - 360;
        end
    end

   % 부드러운 제어를 위해 p gain을 0.4로 설정
    A_control = guide_angle * 0.5;

%         % Saturation
%         if A_control >= 30
%             A_control = 30;
%         end
% 
%         if A_control <= -30
%             A_control = -30;
%         end

    %%%%%%%%%%%%%%%%%%%% 버그2 알고리즘 %%%%%%%%%%%%%%%%%%%
    % 장애물 진입 초기 위치부터 목표 위치까지 일직선을 긋는다
    % 장애물 돌다가 위 직선과 교차하는 점과 만나면 장애물 탈출

    % 자동차로부터 목표 지점까지의 기울기 저장
    target_angle = [target_angle; round(atan2d(S_Map.pointEnd(2) - Robot_State(2), S_Map.pointEnd(1) - Robot_State(1)))];
    count = count + 1;


    if count > 10
        if (round(atan2d(S_Map.pointEnd(2) - Robot_State(2), S_Map.pointEnd(1) - Robot_State(1))) == target_angle(1))
            Goal_x = S_Map.pointEnd(1);
            Goal_y = S_Map.pointEnd(2);

            guide_angle = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);

            if (abs(guide_angle) > 180)
                if (guide_angle < 0)
                    guide_angle = guide_angle + 360;
                else
                    guide_angle = guide_angle - 360;
                end
            end

            A_control = guide_angle;

            detect = 0;
            escape_flag = 1;
            target_angle = [];
            count = 0;
            count_target = 0;
        end
    end


% detect = 0일 경우는 목표 지점을 최종 목적지로 한다.
else
    Goal_x = S_Map.pointEnd(1);
    Goal_y = S_Map.pointEnd(2);

    A_control = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);
    count_target = count_target + 1;
end

% 속도 제어
if detect == 1
    Robot_State(3) = 2;
else
    if(Robot_State(3) > 3)
        V_control = 0; 
    else 
        V_control = 1;
    end
end