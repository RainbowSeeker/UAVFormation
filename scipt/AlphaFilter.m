classdef AlphaFilter
    properties
        cutoff_freq = 0;
        alpha = 0;
        filter_state;
    end
    
    methods
        function obj = AlphaFilter(dt, cutoff_freq, sample)
            if nargin > 0
                obj.alpha = dt / (dt + 1 / (2 * pi * cutoff_freq));
            end
            obj.filter_state = sample;
        end
        
        function filtered = update(obj, sample)
            obj.filter_state = obj.updateCalculation(sample);
            filtered = obj.filter_state;
        end
    end
    
    methods (Access = protected)
        function filtered = updateCalculation(obj, sample)
            filtered = obj.filter_state + obj.alpha * (sample - obj.filter_state);
        end
    end
end