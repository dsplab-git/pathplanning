function [ Flag_Collision ] = PP_Collision_Check( Robot_State_Array, S_Map)     % PP_Collision_Check라는 함수는 Robot_State_Array와 S_Map을 인자로 받아 Flag_Collision을 return 함

Flag_Collision = 0;                                         % Flag_Collision은 0, 1, 2 중 하나의 값을 가질 수 있음
Robot_State = Robot_State_Array(:,end);
map = S_Map.arrMap;
End_Point = S_Map.pointEnd;
Arrive_Condition = S_Map.coditionArrival;

%% Collision Check
if(size(Robot_State_Array,2) == 1)
    return;
end
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
            Flag_Collision = 1;
            return;
        end
    end
end



% 도착지점
if norm(Robot_State(1:2) - End_Point(1:2)) < Arrive_Condition
    Flag_Collision = 2;                                             % 2는 도착을 의미
end
end