%%%%%%%%%%%%%%%%%%%%%%%%% Global Motion Planning %%%%%%%%%%%%%%%%%%%%%%%%%%
% Input = 장애물 정보 ( S_Map.arrMap ) ,  출발지와 목적지 ( S_Map.start_point,S_Map.end_point )
% Output = Mid_Point

Mid_Point = [S_Map.pointEnd(1), S_Map.pointEnd(2)];
Mid_State = 1;
Ref_Speed = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%