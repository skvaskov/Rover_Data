function [sol,user] = test_example(example)
    disp('Extracting inputs')
    f = example.f ;
    p = example.p ;
    x0 = example.x0 ;
    p0 = example.p0(:) ;
    T = example.T ;
    Nt = example.Nt ;
    
    if isfield(example,'test_gradient')
        test_gradient = example.test_gradient ;
    else
        test_gradient = 0 ;
    end
    
    if isfield(example,'run_solver')
        run_solver = example.run_solver ;
    else
        run_solver = 1 ;
    end
    
    if isfield(example,'plot_results')
        plot_results = example.plot_results ;
    else
        plot_results = 0 ;
    end
    
    if isfield(example,'pl')
        pl = example.pl(:) ;
    else
        pl = -Inf*ones(size(p0)) ;
    end
    
    if isfield(example,'pu')
        pu = example.pu(:) ;
    else
        pu = +Inf*ones(size(p0)) ;
    end
    
    [~,Ntrials] = size(x0) ;

    data = cell(1,Ntrials) ;
    input = cell(1,Ntrials) ;
    t = cell(1,Ntrials) ;
    
    if length(T) == 1
        T = repmat(T,1,Ntrials) ;
    end
    
    if length(Nt) == 1
        Nt = repmat(Nt,1,Ntrials) ;
    end
    
    disp('Simulating trials') ;
    tic
    for idx = 1:Ntrials
        tvec = linspace(0,T(idx),Nt(idx)) ;
        uvec = ones(1,Nt(idx)) ;
        [x, ~, ~, ~] = simulateDynamicsWithInput(f,tvec,uvec,x0(:,idx),p) ;
        data{idx} = x ;
        
        if any(isnan(x))
            msg = ['Initial condition ',num2str(idx),...
                   ' resulted in x(t) = NaN when forward-simulated'];
            error(msg)
        end
        if any(isinf(x))
            msg = ['Initial condition ',num2str(idx),...
                   ' resulted in x(t) = inf when forward-simulated'];
            error(msg)
        end 
        
        input{idx} = uvec ;
        t{idx} = tvec ;
    end
    toc

    % model fitter setup
    disp('Setting up nonlinear model fitter')
    tic
    user = nonlinearModelFit(f,t,data,input,p0,'pl',pl,'pu',pu) ;
    toc
    
    % testing equality gradient
    if test_gradient
        disp('Testing numeric vs analytic gradient')
        v = rand(size(user.z0)) ;

        tic
        geqNum = numericJacobian(@user.equalityConstraints,1,v)' ;
        [~,geqAnl] = user.equalityConstraints(v) ;
        toc

        disp(['Numeric vs. Analytic Gradients: ', num2str(norm(geqAnl - geqNum))])
    end

    % run fmincon call
    if run_solver
        [sol, ~] = user.modelFit() ;
        disp(['Parameter Fit: ',mat2str(sol.p)])
        disp(['Fit error: ',mat2str(abs(sol.p - p))])
        disp(['Gradient Norm: ',num2str(norm(sol.grad))])
        disp(['Hessian Norm: ',num2str(norm(sol.hessian))])
    end
    
    if plot_results
        subplot(2,1,1)
        plot(cell2mat(data)')
        title('True Data')
        
        subplot(2,1,2)
        plot(sol.x')
        title('Fitted Model')
    end
end