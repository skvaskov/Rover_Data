disp('Testing numeric vs analytic gradient')
v = rand(size(user.z0)) ;

tic
geqNum = numericJacobian(@user.equalityConstraints,1,v)' ;
[~,geqAnl] = user.equalityConstraints(v) ;
toc

disp(['Numeric vs. Analytic Gradients: ', num2str(norm(geqAnl - geqNum))])