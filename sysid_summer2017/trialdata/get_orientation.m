function [orientation] = get_orientation(U)
%get 3xN vector of roll,pitch,yaw angles from rotation matrix U. Assumes
%Eulrer orientation of yaw,pitch,roll
%U is 3x3xN
sz=size(U);
orientation=zeros(3,sz(3));
for i=1:sz(3)
orientation(3,i)=atan2(U(2,1,i),U(1,1,i));
orientation(2,i)=atan2(-U(3,1,i),sqrt(U(3,2,i)^2+U(3,3,i)^2));
orientation(1,i)=atan2(U(3,2,i),U(3,3,i));
end

end

