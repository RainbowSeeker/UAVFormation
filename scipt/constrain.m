function output = constrain(input, min_val, max_val)
    if input > max_val
        output = max_val;
    else
        if input < min_val
            output = min_val;
        else
            output = input;
        end
    end
end

