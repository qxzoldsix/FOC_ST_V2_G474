function command=firmware_inverse_park(vdq,cs)
command=[vdq(1)*cs(1)-vdq(2)*cs(2);vdq(2)*cs(1)+vdq(1)*cs(2)];
end
