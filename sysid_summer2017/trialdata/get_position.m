function [position] = get_position(initial_points,R,t)
%R=(3x3xN_timesteps),t=3XNtime_steps %initial=3xNpoints
%position=3xNpointsxNtimesteps
szR=size(R);
szi=size(initial_points);
position=zeros(3,szi(2),szR(3));
position(:,:,1)=initial_points;
for i=2:szR(3)
    position(:,:,i)=R(:,:,i)*position(:,:,i-1)+t(:,i);
end

end

