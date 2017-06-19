function [velocity] = get_velocities(CPos,R,time)
%positions 3XNtimesteps 
sz=size(positions);
velocity=zeros(sz);
dS=positions(:,2)-positions(:,1);
dt=time(2)-time(1);
velocity(:,:,1)=R(:,:,2)'*dS/dt;
for i=2:sz(3)-1
dS=positions(:,i+1)-positions(:,i-1);
dt=time(i+1)-time(i-1);
velocity(:,:,i)=R(:,:,i)'*dS/dt;
end
dS=positions(:,end)-positions(:,end-1);
dt=time(end)-time(end-1);
velocity(:,:,end)=R(:,:,end)'*dS/dt;
end

