classdef (StrictDefaults) VelocitySmoothing < matlab.System
    %  Generates rate and setpoint trajectory.
    %    |T1| T2 |T3|
    %     ___
    %   __| |____   __ Jerk
    %            |_|
    %        ___
    %       /   \	 Acceleration
    %   ___/     \___
    %             ___
    %           ;"
    %          /
    %         / 	 Velocity
    %        ;
    %   ----"
    %
    % NOTE: When renaming the class name untitled5, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.

    % Public, tunable properties
    properties
        % Maximum jerk allowed [m/s³]
        max_jerk = 22
        % Maximum vertical acceleration allowed [m/s²]
	    max_accel = 8
        % Climb rate produced by max allowed throttle [m/s]
        max_climb_rate = 2
        % Maximum sink rate (with min throttle, max speed) [m/s]
        max_sink_rate = 2
    end

    % Public, non-tunable properties
    properties (Nontunable)
        %SampleTime Sample time
        %   The sample time for any variable-size signal must be discrete
        %
        %   Default:         -1 (inherit)
        SampleTime         = -1
        
    end

    % Pre-computed constants
    properties (Access = private)
        % Input 
	    vel_sp

	    % State (previous setpoints) 
	    state struct

	    direction

	    % Initial conditions 
	    state_init struct

	    % Duration of each phase 
	    T1 % Increasing acceleration [s]
	    T2 % Constant acceleration [s]
	    T3 % Decreasing acceleration [s]

	    local_time % Current local time
        
        is_init logical = false
    end

    methods
        % Constructor
        function obj = VelocitySmoothing(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:}, 'SampleTime');
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj, v0x0, ~, ~)
            % Perform one-time calculations, such as computing constants
            m_zero = cast(0, 'like', v0x0);
            obj.state = struct('j', m_zero, 'a', m_zero, 'v', m_zero, 'x', m_zero);
            obj.state_init = obj.state;
        end

        function [vel, pos] = stepImpl(obj, v0x0, vel_sp, reset)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if reset > 0 || obj.is_init ~= true
                obj.is_init = true;
                resetState(obj, v0x0(1), v0x0(2));
            end

            % Update durations
            updateDurations(obj, vel_sp);
            % Update state
            updateTraj(obj);
            vel = obj.state.v;
            pos = obj.state.x;
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function resetState(obj, vel, pos)
            obj.state.j = cast(0, 'like', obj.state.j);
            obj.state.a = cast(0, 'like', obj.state.a);
            obj.state.v = cast(vel, 'like', obj.state.v);
            obj.state.x = cast(pos, 'like', obj.state.x);

            obj.state_init = obj.state;
        end

        function updateDurations(obj, vel_setpoint)
            obj.vel_sp = max(-obj.max_sink_rate, min(vel_setpoint, obj.max_climb_rate));
            obj.local_time = 0;
            obj.state_init = obj.state;

            computeDirection(obj);
            
            if obj.direction ~= 0
		        updateDurationsMinimizeTotalTime(obj);
	        else 
		        obj.T1 = cast(0, 'like', obj.T1);
                obj.T2 = cast(0, 'like', obj.T2);
                obj.T3 = cast(0, 'like', obj.T3);
            end
        end

        function computeDirection(obj)
            % Compute the velocity at which the trajectory will be when the acceleration will be zero
            vel_zero_acc = obj.state.v; % v
            if abs(obj.state.a) > eps('like', obj.state.a)
                j_zero_acc = -sign(obj.state.a) * obj.max_jerk; % Required jerk to reduce the acceleration
                t_zero_acc = -obj.state.a / j_zero_acc; % Required time to cancel the current acceleration
                vel_zero_acc = obj.state.v + obj.state.a * t_zero_acc + 0.5 * j_zero_acc * t_zero_acc * t_zero_acc;
            end
            
            % Depending of the direction, start accelerating positively or negatively
            obj.direction = sign(obj.vel_sp - vel_zero_acc);
            if obj.direction == 0
                obj.direction = sign(obj.state.a);
            end
        end

        function updateDurationsMinimizeTotalTime(obj)
            jerk_max_T1 = obj.max_jerk * obj.direction;
            delta_v = obj.vel_sp - obj.state.v;
            
            % compute increasing acceleration time
	        obj.T1 = VelocitySmoothing.computeT1(obj.state.a, delta_v, jerk_max_T1, obj.max_accel);
        
	        % compute decreasing acceleration time
	        obj.T3 = VelocitySmoothing.computeT3(obj.T1, obj.state.a, jerk_max_T1);
        
	        % compute constant acceleration time
	        obj.T2 = VelocitySmoothing.computeT2(obj.T1, obj.T3, obj.state.a, delta_v, jerk_max_T1);

        end

        function updateTraj(obj)
            obj.local_time = obj.local_time + obj.SampleTime;
            t_remain = obj.local_time;
        
            t1 = min(t_remain, obj.T1);
            obj.state = VelocitySmoothing.evaluatePoly(obj.max_jerk, obj.state_init.a, obj.state_init.v, obj.state_init.x, t1, obj.direction);
            t_remain = t_remain - t1;
        
            if t_remain > 0
                t2 = min(t_remain, obj.T2);
                obj.state = VelocitySmoothing.evaluatePoly(0, obj.state.a, obj.state.v, obj.state.x, t2, 0);
                t_remain = t_remain - t2;
            end
        
            if t_remain > 0
                t3 = min(t_remain, obj.T3);
                obj.state = VelocitySmoothing.evaluatePoly(obj.max_jerk, obj.state.a, obj.state.v, obj.state.x, t3, -obj.direction);
                t_remain = t_remain - t3;
            end
        
            if t_remain > 0
                obj.state = VelocitySmoothing.evaluatePoly(0, 0, obj.state.v, obj.state.x, t_remain, 0);
            end
        end
        
        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end
        
        function validateInputsImpl(~, v0x0, vel_sp, ~)
            % Validate inputs to the step method at initialization
            
            % Validate v0x0
            validateattributes(v0x0, {'single', 'double'}, {'nonnan', ...
                'real', 'column', 'nrows', 2, 'finite', 'nonempty'}, mfilename, 'v0x0');
            % Validate vel_sp
            validateattributes(vel_sp, {'single', 'double'}, {'nonnan', ...
                'real', 'finite', 'scalar', 'nonempty'}, mfilename, 'vel_sp');

            % Input datatypes should be the same
            isDataTypeEqual = isequal(class(v0x0), class(vel_sp));           
            coder.internal.errorIf(~isDataTypeEqual, ...
                'dataTypeMismatch', ...
                class(v0x0), class(vel_sp));
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(~)
            % Define total number of inputs
            num = 3;
        end

        function [sz1, sz2] = getOutputSizeImpl(obj)
            % Return size for each output port
            sz1 = [1 1];
            sz2 = [1 1];
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function varargout = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            varargout{1} = propagatedInputDataType(obj,1);
            varargout{2} = propagatedInputDataType(obj,1);
        end

        function varargout = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            varargout{1} = false;
            varargout{2} = false;
        end

        function varargout = isOutputFixedSizeImpl(~, ~)
            varargout{1} = true;
            varargout{2} = true;
        end
        

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = ["Velocity","Smoothing"]; % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end

        function sts = getSampleTimeImpl(obj)
            if obj.SampleTime < 0
                sts = createSampleTime(obj,'Type','Inherited');
            else
                sts = createSampleTime(obj,'Type','Discrete',...
                    'SampleTime',obj.SampleTime);
            end
        end
    end

    methods (Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(...
                'Title','Parameters',...
                'PropertyList',{'max_jerk','max_accel', 'max_climb_rate', 'max_sink_rate', 'SampleTime'});
        end
        %% 依赖函数
        function traj = evaluatePoly(j, a0, v0, x0, t, d)
            jt = cast(d * j, 'like', v0);
            t2 = t * t;
            t3 = t2 * t;
        
            traj.j = jt;
            traj.a = a0 + jt * t;
            traj.v = v0 + a0 * t + 0.5 * jt * t2;
            traj.x = x0 + v0 * t + 0.5 * a0 * t2 + 1 / 6 * jt * t3;
        end

        function T1 = computeT1(a0, v3, j_max, a_max)
            delta = 2 * a0 * a0 + 4 * j_max * v3;
            
            T1 = cast(0, 'like', a0);
            if delta < 0
                % Solution is not real
                
                return;
            end
        
            sqrt_delta = sqrt(delta);
            T1_plus = (-a0 + 0.5 * sqrt_delta) / j_max;
            T1_minus = (-a0 - 0.5 * sqrt_delta) / j_max;
        
            T3_plus = a0 / j_max + T1_plus;
            T3_minus = a0 / j_max + T1_minus;
        
            if T1_plus >= 0 && T3_plus >= 0
                T1 = T1_plus;
            elseif T1_minus >= 0 && T3_minus >= 0
                T1 = T1_minus;
            end
        
            T1 = VelocitySmoothing.saturateT1ForAccel(a0, j_max, T1, a_max);
        
            T1 = max(T1, 0);
        end

        function T2 = computeT2(T1, T3, a0, v3, j_max)
            T2 = cast(0, 'like', a0);
        
            den = a0 + j_max * T1;
        
            if abs(den) > eps
                T2 = (-0.5 * T1 * T1 * j_max - T1 * T3 * j_max - T1 * a0 + 0.5 * T3 * T3 * j_max - T3 * a0 + v3) / den;
            end
        
            T2 = max(T2, 0);
        end

        function T3 = computeT3(T1, a0, j_max)
            T3 = max(a0 / j_max + T1, 0);
        end

        function T1_new = saturateT1ForAccel(a0, j_max, T1, a_max)
            % Check maximum acceleration, saturate and recompute T1 if needed
            accel_T1 = a0 + j_max * T1;
            T1_new = T1;
            
            if accel_T1 > a_max
                T1_new = (a_max - a0) / j_max;
            elseif accel_T1 < -a_max
                T1_new = (-a_max - a0) / j_max;
            end
        end
    end


    methods
        %------------------------------------------------------------------
        function set.SampleTime(obj, st)
            validateattributes( st, {'numeric'}, ...
                {'scalar', 'real', 'nonnan', 'finite', 'nonzero'}, ...
                class(obj), 'SampleTime');
            
            % Only inherited and discrete sample time are supported for
            % variable-size signal inputs
            coder.internal.errorIf((st < 0) && (st ~=-1), ...
                'negativeSampleTime');
            
            obj.SampleTime = st;
        end
    end
end
