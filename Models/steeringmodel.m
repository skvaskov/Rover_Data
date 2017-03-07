function [delta] = steeringmodel(u1)
u1=u1+1500;
p2=[-3.73639640971342e-07,-0.000143441656596508,1.13415861973086];
p1=[-0.00126697093649443,1.94203964249574];
delta=polyval(p2,u1);
% delta=zeros(1,length(u1));
% ppos=[1.36246530750908e-06,-0.00210008777752679,0.170711470355914];
% pneg=[-1.36246530750907e-06,-0.00210008777752678,-0.170711470355914];
% for i=1:length(u1)
%     if abs(u1)<=80
%         delta(i)=0;
%     elseif u1>80
%             delta(i)=polyval(ppos,u1(i));
%     else 
%            delta(i)=polyval(pneg,u1(i));
%     end
% end



end

