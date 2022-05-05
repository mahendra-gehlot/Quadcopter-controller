function Integrated = integration(rg,rd,rgo, rdo, Io, i, h)
if i == 1
   Integrated = h*(rg - rd)/2;
elseif i == 2
   Integrated = h*( (rgo - rdo) + (rg - rd) )/2;
else
   Integrated = ( (rg - rd) + 2/h*Io + (rgo - rdo) )*h/2;
end
end