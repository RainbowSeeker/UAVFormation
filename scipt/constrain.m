function output = constrain(input, min_val, max_val)
    if input > max_val
        output = cast(max_val, 'like', input);
    else
        if input < min_val
            output = cast(min_val, 'like', input);
        else
            output = cast(input, 'like', input);
        end
    end
end

