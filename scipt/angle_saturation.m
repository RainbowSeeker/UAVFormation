function angle = angle_saturation(angle_in)
angle_in = rem(angle_in, 2*pi);
if abs(angle_in) > pi
    angle = angle_in - 2*pi*sign(angle_in);
else
    angle = angle_in;
end
end