function psi = psi_saturation(psi_in)
psi_in = rem(psi_in, 2*pi);
if abs(psi_in) > pi
    psi = psi_in - 2*pi*sign(psi_in);
else
    psi = psi_in;
end
end