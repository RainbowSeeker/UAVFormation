function wrapped = wrapTo2Pi(x)
    low = cast(0, 'like', x);
    high = cast(2*pi, 'like', x);
    range = high - low;
    
    if x < low
        x = x + range * ((low - x) / range + 1);
    end
    
    wrapped = low + mod(x - low, range);
end