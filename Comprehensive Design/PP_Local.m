%%%%%%%%%%%%%%%%%%%%%%%% Local Path Planning %%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% 현재 위치 (Robot_State(1), Robot_State(2))
% 목표 지점 (pointEnd(1), pointEnd(2))
% 장애물 정보는 구조체(S_Map)의 arrMap에 저장되어 있음
% Sensor 정보는 LRF 변수에 저장되어 있음

%% Initial Value
if(initial_flag == 0)

end

% 방향 결정
A_control = atan2d(Goal_y - Robot_State(2), Goal_x - Robot_State(1)) - Robot_State(4);
% 속도 결정
if(Robot_State(3) > Ref_Speed)
    V_control = 0;
else
    V_control = 1;
end