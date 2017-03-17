classdef nonlinearModelFit
   properties
    % The user enters the following properties:
        fdyn      % system dynamics and derivatives wrt states and params
        
        data      % state data cell array (1 x Ntrials), where each trial
                  % is of size (Nstates x <time steps>)
        
        input     % input data cell array (1 x Ntrials), where each input
                  % stream is of size (Ninputs x <time steps>)
        
        p0        % initial guess at parameters (Nparams x 1)
        
        t         % cell array of time vectors for each trial (1 x Ntrials)
        
    % The user can enter the following properties as varargin
        pl        % parameter lower bound (Nparams x 1)
        
        zl        % state lower bound (Nstates x 1)
        
        pu        % parameter upper bound (Nparams x 1)
        
        zu        % state upper bound (Nstates x 1)
        
        scaling   %Nstatex1 matrix with scaling factors for cumpting the cost for each state
        
        
        x2track   % vector of length <= Nstates that lists what states to
                  % track for the optimization cost function
                  
        verbose   % flag to display fmincon info

    % These properties are created and used internally
        Nstates   % number of states of system

        Nparams   % number of unknown parameters to identify
        
        Ndata     % vector of length Ntrials, where each element is
                  % the number of data points for each trial
                  
        Ntrials   % number of trials input by user
        
        z0        % initial condition of decision variable z
        
        lb        % lower bound for z
    
        ub        % upper bound for z
        
        dt        % time step between data points (T/Ndata)
        
        x2omit    % inverse of x2track
   end
   
   methods
    % CONSTRUCTOR
        function user = nonlinearModelFit(systemdynamics,...
                        t_in,data_in,input_in,p0,varargin)
                    
           % Determine number of trials and length of each trial
           Ntrials = length(data_in) ;
           Ndata = zeros(1,Ntrials) ;
           for idx = 1:Ntrials
               Ndata(idx) = length(data_in{idx}) ;
           end
           
           % Restructure data, input, and time
           data = cell2mat(data_in) ;
           input = cell2mat(input_in) ;
           t = cell2mat(t_in) ;
           dt = [diff(t_in{1}),0] ;
           for idx = 2:Ntrials
               dt = [dt, diff(t_in{idx}), 0] ;
           end
           
           % Catch optional inputs
           nVarargs = length(varargin) ;
           
           if nVarargs > 0
               for argidx = 1:2:nVarargs
                   switch varargin{argidx}
                       case 'x2track'
                           x2track = varargin{argidx+1} ;
                       case 'pl'
                           pl = varargin{argidx+1} ;
                       case 'pu'
                           pu = varargin{argidx+1} ;
                       case 'zu'
                           zu = varargin{argidx+1} ;
                       case 'zl'
                           zl = varargin{argidx+1};
                       case 'scaling'
                           scaling=varargin{argidx+1};
                       case 'verbose'
                           verbose = varargin{argidx+1} ;
                   end
               end
           end
           
           % If the optional inputs weren't provided, create them. First,
           % make the list of states to track (track all states by default)
           [Nstates,Nsteps] = size(data) ;
           Nparams = length(p0);
           allstates = 1:Nstates ;
           if ~exist('x2track','var')
               x2track = allstates ;
           end
           x2omit = allstates(~ismember(allstates,x2track)) ;
           
           if ~exist('verbose','var')
               verbose = 0 ;
           end
           
      
           % Create 'user' object
           user.fdyn = systemdynamics ;
           user.Nstates = Nstates ;
           user.Nparams = Nparams ;
           user.data = data ;
           user.input = input ;
           user.Ndata = Ndata ;
           user.Ntrials = Ntrials ;
           user.p0 = p0(:) ;
           user.t = t ;
           user.x2track = x2track ;
           user.x2omit = x2omit ;
           user.dt = dt ;
           user.verbose = verbose ;
           
           % Create optimization initial condition
           [x, ~, ~, ~] = user.forwardSimulateDynamics(p0) ;
           z0 = [x(:); p0(:)] ;
           user.z0 = z0 ;
           
           % Create optimization bounds
           Nbounds = length(x(:)) ;
                % If bounds are not provided, leave the parameters and states unbounded by
           % default
           if ~exist('pl','var')
               pl = -Inf*ones(Nparams,1) ;
           end
           if ~exist('pu','var')
               pu = +Inf*ones(Nparams,1) ;
           end
           
           if ~exist('zl','var')
               zl = -Inf*ones(Nbounds,1) ;
           else
               zl = repmat(zl,1,Nsteps) ;
               zl = zl(:);
           end
           
           if ~exist('zu','var')
               zu = +Inf*ones(Nbounds,1) ;
           else
               zu = repmat(zu,1,Nsteps) ;
               zu = zu(:);
           end
           if ~exist('scaling','var')
                scaling=ones(Nstates,1);
            end
           user.scaling=scaling;
           lb = [zl; pl] ;
           ub = [zu; pu] ;
           user.pl = pl(:) ;
           user.pu = pu(:) ;
           user.zl = zl(:) ;
           user.zu = zu(:) ;
           user.lb = lb(:) ;
           user.ub = ub(:) ;
        end
        
    % OPTIMIZATION CALL
        function [solution, problem] = modelFit(user,max_iterations)
            if nargin < 2
                max_iterations = 100000 ;
            end

            
            % Set up options
            if user.verbose
                options = optimoptions('fmincon','GradObj','on','GradCons','on',...
                                       'Display','iter','Algorithm','interior-point' ) ;
            else
                options = optimoptions('fmincon','GradObj','on','GradCons','on',...
                                       'Algorithm','interior-point' ) ;
            end
            options.MaxIterations = max_iterations ;
            options.MaxFunctionEvaluations= 2*user.Nstates*max_iterations;
            
            % Create problem object to pass to fmincon
            problem.objective = @user.cost ;
            problem.x0 = user.z0 ;
            problem.lb = user.lb ;
            problem.ub = user.ub ;
            problem.nonlcon = @user.nonlinearConstraints ;
            problem.solver = 'fmincon' ;
            problem.options = options ;
            
            [z,fval,exitflag,output,lambda,grad,hessian] = fmincon(problem) ;
            [x,p] = user.statedecode(z) ;
            solution.x = x ;
            solution.p = p ;
            solution.finalCost = fval ;
            solution.exitflag = exitflag ;
            solution.output = output ;
            solution.lambda = lambda ;
            solution.grad = grad ;
            solution.hessian = hessian ;
        end

    % METHODS TO USE IN OPTIMIZATION
        function [c,gc] = cost(user,z)
            % Calculate the cost associated with the decision variable vector
            % z, along with the gradient of the cost, at a single optimization
            % step. The vector z is a column vector of length (Ns*Nd + Np)

            % Extract states (Nstates x Ndata) and parameters (Nparams x 1)
            [x,~] = user.statedecode(z) ;
            
            %scale both simulated states and data
            n=sum(user.Ndata);
            s=repmat(user.scaling,1,n);
            % Zero out the states that don't need to be tracked
            x(user.x2omit,:) = 0 ;
            d=user.data;
            sx=s.*x;
            sd=s.*d;
            c = sum((sx(:)-sd(:)).^2) ;
            gc = [2*(sx(:)-sd(:))', zeros(1,user.Nparams)] ;
        end

        function [cons,eq,gcons,geq] = nonlinearConstraints(user,z)
        % Given the decision variable vector z, set up the optimization
        % constraints and the gradient of these constraints

        % Set inequality constraints to null
            cons = [] ;
            gcons = [] ;
        % Get equality constraints
            [eq, geq] = user.equalityConstraints(z) ;
        end
        
        function [eq,geq] = equalityConstraints(user,z)
        % Given the decision variable vector z, calculate the equality
        % constraints and the equality constraint gradient using the
        % user-provided system dynamics
        
        % Extract states (Nstates x Ndata) and parameters (Nparams x 1)
            Ns = user.Nstates ;
            Np = user.Nparams ;
            Nd = user.Ndata ;
          
            N = sum(Nd) ;
            [x,p] = user.statedecode(z) ;
            
        % Evaluate the system dynamics and get the relevant gradients
            [dxdt, dfdx, dfdp] = user.evaluateDynamics(x,p);

        % Calculate the equality constraint violations
            % Enforce that x(k+1) = x(k) + f(x(k)).*dt(k)
            eq = x(:,2:N) - x(:,1:N-1) - repmat(user.dt(1:N-1),Ns,1).*dxdt(:,1:N-1) ;
            
            
            % Zero out the beginning of each trial?
            Ncum = cumsum(user.Ndata) ;
            eq(:,Ncum(1:end-1)) = 0 ;
            
            % add zero padding at the beginning for the first trial's initial
            % condition
            eq = [zeros(Ns,1), eq] ;
            
%              %add equality constraints for initial conditions of each trial
              Ndt=cumsum(Nd);
              init=[1,Ndt(1:end-1)+1];
              eqinit=x(:,init)-user.data(:,init);
              eq=[eqinit,eq];
              
            %transform eq into row vector
            eq=eq(:);
            
            %Add constraints enforcing that the initial condition for each
            %trial are equal to the data
            
        % Calculate the equality constraint gradient           
            % Reshape dfdx
            dtdfdx = repmat(permute(-user.dt,[3 1 2]),Ns,Ns).*dfdx ;
            dfdxcel = num2cell(dtdfdx,[1 2]) ;
            dfdxgeq = blkdiag(dfdxcel{1:end-1}) ;
            
            
            % Reshape dfdp
            dtdfdp = repmat(permute(-user.dt,[3 1 2]),Ns,Np).*dfdp ;
            dfdpgeq = reshape(permute(dtdfdp,[1 3 2]),[],Np) ;
            
            % Determine size needed for gradient matrix (Neq x (Ns*N + Np))
            Neq = length(eq)-length(init)*Ns; %leave equality constaints for initial conditions out for now
            
            % Generate equality constraint gradient
            geq = [eye(Neq), [zeros(Ns,Np); dfdpgeq(1:Neq-Ns,:)]]' + ...
                  [zeros(Ns,Neq+Np); -eye(Neq-Ns)+dfdxgeq, zeros(Neq-Ns,Np+Ns)]' ;
            
            % Zero out the columns of geq corresponding to the first time
            % step of each trial
            col2zero = repmat([1,Ns.*Ncum(1:end-1)+1],Ns,1) + ...
                       repmat((1:Ns)'-1,1,length(Ncum)) ;
            geq(:,col2zero(:)) = 0 ;
            
            %add initial condition constraint to equality gradient matrix
             temp=[];
             initgeq=[eye(Ns);zeros(Ns*(N-1)+Np,Ns)];
             
            for i=2:length(init)
                temp=[zeros((init(i)-1)*Ns,Ns);eye(Ns);zeros(Ns*(N-init(i))+Np,Ns)];
                initgeq=[initgeq,temp];
            end
            geq=[initgeq,geq];
            
        end

    % UTILITY FUNCTIONS
        function [dxdt, dfdx, dfdp] = evaluateDynamics(user,x,p)
        % Given the system state x and parameters p, evaluate the system
        % dynamics and return dx/dt and the gradients of the dynamics
        % with respect to x and p (df/dx and df/dp)
            u = user.input ;
            Ns = user.Nstates ;
            Np = user.Nparams ;
            Nd = sum(user.Ndata) ;
            
            % set up matrices to fill in
            dxdt = zeros(Ns,Nd) ;
            dfdx = zeros(Ns,Ns,Nd) ;
            dfdp = zeros(Ns,Np,Nd) ;
            
            % evaluate at first datum
            [dxdtPoint,dfdxPoint,dfdpPoint] = user.fdyn(x(1:Ns,1),u(:,1),p) ;
            dxdt(:,1) = dxdtPoint ;
            dfdx(:,:,1) = dfdxPoint ;
            dfdp(:,:,1) = dfdpPoint ;
            
            % variable to track which trial is being evaluated
            trialCounter = 1 ;
            
            for idx = 2:Nd
                % evaluate the system dynamics at each data point
                [dxdtPoint,dfdxPoint,dfdpPoint] = user.fdyn(x(:,idx),u(:,idx),p) ;
                
                if idx == user.Ndata(trialCounter)
                    % if we have hit the end of a trial,
                    % leave the gradients as zero
                    trialCounter = trialCounter + 1 ;
                else
%                     % fill in the matrices!
                    dxdt(:,idx) = dxdtPoint ;
                    dfdx(:,:,idx) = dfdxPoint ;
                    dfdp(:,:,idx) = dfdpPoint ;
                end
                    
                
            end
        end
        
       function [x, dxdt, dfdx, dfdp] = forwardSimulateDynamics(user,p)
        % This is nearly the same as evaluate dynamics, but x is not
        % provided; for some system parameters p, simulate the system and
        % return the trajectory, dx/dt, and the gradients of the dynamics
        % with respect to x and p (df/dx and df/dp)
            u = user.input ;
            Ns = user.Nstates ;
            Np = user.Nparams ;
            Nd = sum(user.Ndata) ;
            trialCounter = 1 ;
            
            % set up matrices to fill in
            x = user.data ;
            dxdt = zeros(Ns,Nd) ;
            dfdx = zeros(Ns,Ns,Nd) ;
            dfdp = zeros(Ns,Np,Nd) ;
            
            % evaluate at initial conditions
            [dxdtPoint,dfdxPoint,dfdpPoint] = user.fdyn(x(1:Ns,1),u(:,1),p) ;
            dxdt(:,1) = dxdtPoint ;
            dfdx(:,:,1) = dfdxPoint ;
            dfdp(:,:,1) = dfdpPoint ;
            Ndt=cumsum(user.Ndata);
            for idx = 2:Nd
                    if idx == Ndt(trialCounter) + 1
                        % if we have hit the start of a trial, don't
                        % forward-simulate the dynamics
                        trialCounter = trialCounter + 1 ;
                    else
                        x(:,idx) = x(:,idx-1) + user.dt(idx-1).*dxdtPoint ;
                    end

                % evaluate the system dynamics for the next time step
                [dxdtPoint,dfdxPoint,dfdpPoint] = user.fdyn(x(:,idx),u(:,idx),p) ;
                dxdt(:,idx) = dxdtPoint ;
                dfdx(:,:,idx) = dfdxPoint ;
                dfdp(:,:,idx) = dfdpPoint ;
            end
        end

        function [x,p] = statedecode(user,z)
        % Given the decision variables vector z, return the state of the
        % system and the current parameters
           Ns = user.Nstates;
           Np = user.Nparams;
           Nd = user.Ndata;
           Ndt = cumsum(Nd);
           x = reshape(z(1:end-Np),Ns,sum(Nd));
           %augment z with initial points
%            x=[user.data(:,1),x];
%            for i=1:(length(Ndt)-1)
%                x=[x(:,1:Ndt(i)),user.data(:,Ndt(i)+1),x(:,Ndt(i)+1:end)];
%            end
           
           p = z(end-Np+1:end);
        end

        function z = stateencode(user,x,p)
        % Given the state at and the current parameters p, return an
        % updated decision variable vector z.
%            Nd = user.Ndata;
%            Ndt = cumsum(Nd);
%            %remove initial conditions from x
%            x=x(:,2:end);
%             for i=1:(length(Ndt)-1)
%             x=[x(:,1:Ndt(i)-i),x(:,Ndt(i)-i+2:end)];
%            end
           z = [x(:);p] ;
        end
    end
end
