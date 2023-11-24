Distance = LRF(:,2);
Distance_out = LRF;
Distance_out2 = LRF;
Distance_out3 = LRF;
Fsize = 10;
for i = Fsize+1 : length(Distance)-Fsize-1
    arrGap = Distance(i-Fsize : i+Fsize);
    minvalue = min(arrGap);
    Distance_out(i,2) = minvalue;
end

Fsize = 10;
for i = Fsize+1 : length(Distance)-Fsize-1
    arrGap = Distance(i-Fsize : i+Fsize);
    minvalue = min(arrGap);
    Distance_out2(i,2) = minvalue;
end

Distance_out3(:,2) = Distance_out3(:,2)-Fsize;
figure(3)
polarplot((LRF(:,1) + Robot_State(4))*pi/180,LRF(:,2)','r')
hold on
polarplot((Distance_out(:,1) + Robot_State(4))*pi/180,Distance_out(:,2)','b')
% polarplot((Distance_out2(:,1) + Robot_State(4))*pi/180,Distance_out2(:,2)','g')
polarplot((Distance_out3(:,1) + Robot_State(4))*pi/180,Distance_out3(:,2)','g')
hold off
