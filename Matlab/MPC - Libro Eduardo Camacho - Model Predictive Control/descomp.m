function [B,A,d] = descomp(FT)
% By:  Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
%[B,A,d] = descomp(FT)
%descompone uma função de transferencia em numerador (B) denominador
%(A) e atraso (d) em tempo discreto. (Retorna CELDAS)
    [B,A]=tfdata(FT,'v');  %Numerador e denominador do processo
    d=FT.iodelay;
end