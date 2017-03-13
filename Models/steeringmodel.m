function [delta] = steeringmodel(u1)
u1=u1-80;
p=[-3.67478776615971e-10,-4.59218815521785e-07,-0.00126740525229302,-0.0204632799650850];
delta=polyval(p,u1);
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

if u1==1580
    delta=0;
end

end

