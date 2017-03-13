%calculations to determine center of rover (all in m)
%Acryclic dimensions

length=.47;
width=.345;

%ball postions bij (i is the cluster 1=passenger front, 2=driver front,
%3=driver rear) j is the ball for that cluster (1=closest to corner,
%2=closest to front, 3=closest to rear). %ci is the centroid of each cluster
%0,0 position is driver front corner


%cluster 1
b11=[.47/2-.01376;-.345/2+.01349];
b12=[.47/2-.01503;-.345/2+.06885];
b13=[.47/2-.06462;-.345/2+.01430];
c1=(b11+b12+b13)/3;
rc1=-c1;
%cluster 2
b21=[.47/2-.01676;.345/2-.01427];
b22=[.47/2-.01693;.345/2-.06885];
b23=[.47/2-.07824;.345/2-.01210];
c2=(b21+b22+b23)/3;
rc2=-c2;

%cluster 3
b31=[-.47/2+.01624;.345/2-.01554];
b32=[-.47/2+.06983;.345/2-.01669];
b33=[-.47/2+.01774;.345/2-.06509];
c3=(b31+b32+b33)/3;
rc3=-c3;
% plot([b11(1),b12(1),b13(1),b11(1)],[b11(2),b12(2),b13(2),b11(2)],'r')
% hold on
% plot([b21(1),b22(1),b23(1),b21(1)],[b21(2),b22(2),b23(2),b21(2)],'b')
% plot([b31(1),b32(1),b33(1),b31(1)],[b31(2),b32(2),b33(2),b32(2)],'g')
% plot([c1(1),c2(1),c3(1)],[c1(2),c2(2),c3(2)],'k*')
% axis([-.3,.3,-.2,.2])

%vector from 3 to 2
r23=c2-c3;
%angle between vector connecting c2 and c3
gamma=atan2(r23(2),r23(1));
r13=c1-c3;
chi=atan2(r13(2),r13(1));








 