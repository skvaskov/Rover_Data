function [accel] = get_acceleration(velocity,angV,time)

accel(:,1)=(velocity(:,2)-velocity(:,1))/(time(2)-time(1))+cross(angV(:,1),velocity(:,1));

for i=2:length(time)-1
    accel(:,i)=(velocity(:,i+1)-velocity(:,i-1))/(time(i+1)-time(i-1))+cross(angV(:,i),velocity(:,i));
end

accel(:,length(time))=(velocity(:,end)-velocity(:,end-1))/(time(end)-time(end-1))+cross(angV(:,end),velocity(:,end));


end

