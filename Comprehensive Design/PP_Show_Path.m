function PP_Show_Path( Robot_State_Array, S_Map, S_Sensor, N_figure )
% PP_Show_Path라는 함수는 Robot_State_Array, S_Map, N_figure를 인자로 받고 return 하지않는 void 함수인듯

    %% Load Map info
    map  = S_Map.arrMap;
    start_point = S_Map.pointStart;
    end_point = S_Map.pointEnd;
    
    %% Robot Info
    robot_size = S_Map.sizeRobot;
    
    %% Show Path
    Robot_State = Robot_State_Array(:,end);

    figure(N_figure)
    subplot(1,2,1)
    L = 3;
    imshow(map)
    hold on
    % start
    scatter(start_point(2),start_point(1),(2*L)^2,'or');
    % path
    plot(Robot_State_Array(2,1:end),Robot_State_Array(1,1:end),'k.-')
    % present
    plot(robot_size * sin(0:pi/10:2*pi) + Robot_State(2),robot_size * cos(0:pi/10:2*pi) + Robot_State(1),'b');
    plot([Robot_State(2),1.5*robot_size*sind(Robot_State(4))+Robot_State(2)],[Robot_State(1),1.5*robot_size*cosd(Robot_State(4))+Robot_State(1)],'r');
    scatter(Robot_State(2),Robot_State(1),(2*L)^2,'b');
    % end
    scatter(end_point(2),end_point(1),(2*L)^2,'xr');
    plot(S_Map.coditionArrival * sin(0:pi/10:2*pi) + end_point(2),S_Map.coditionArrival * cos(0:pi/10:2*pi) + end_point(1),'r')
    hold off
    subplot(1,2,2)
    polarplot((S_Sensor.lrfData(:,1) + Robot_State(4))*pi/180,S_Sensor.lrfData(:,2)')                             % LRF 원형 그래프를 보여주기 위해!
    drawnow   
   
end