function [ Robot_State_output ] = PP_Robot_Control( Robot_State, Control_Input, Mode, Robot_Size)      
% PP_Robot_Control이라는 함수는 Robot_State, Control_Input, Mode 인자를 받고 Robot_State_output을 return 함

% Mode 1 : x,y,dx,dy
% robot_state = [x,y,v,theta]

% Mode 2 : x,y,v,theta
% robot_state = [x,y,v,theta]

% Mode 3 : x,y,vl,vr
% robot_state = [x,y,v,theta]

% Robot Size
if nargin < 4
    Robot_Size = 10;
end

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
        d_theta = (Control_Input(1) - Control_Input(2))/Robot_Size;
        Robot_State_output(3) = Robot_State(3) + v;
        Robot_State_output(4) = Robot_State(4) + d_theta;
        Robot_State_output(1) = Robot_State(1) + Robot_State(3)*cosd(Robot_State(4));
        Robot_State_output(2) = Robot_State(2) + Robot_State(3)*sind(Robot_State(4));
end

end