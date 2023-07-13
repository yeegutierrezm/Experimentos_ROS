% By:  Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
% Función encargada de obtener la matriz B (numerador MIMO) y la matriz A
% (denominador MIMO) para el calculo de las difantinas en el MPC. Como
% parametros de entrada debe ser ingresado una celda (Bn) que contenga
% todos los numeradores de la planta y una celda (An) que contenga todos
% los denominadores de la planta. Una forma rápida de obtener esas celdas
% es utilizando la funcion "descomp"
% [B,A] = BA_MIMO(Bn,An)
% B    = Matriz B
% A    = Matriz A (minimo común Multiplo)
% Bn   = Matriz CELDA que contiene los numeradores de la FT MIMO
% An   = Matriz CELDA que contiene los denominadores de la FT MIMO
function[B,A] = BA_MIMO(Bn,An)
  [p,m]=size(An); %Numero de salidas(p) y numero de entradas(m)
  
  %Elimina el cero q aparece en el numerador
    for i=1:p
        for j=1:m
            if Bn{i,j}(1)==0
                Bn{i,j}=Bn{i,j}(2:end);
            end
        end
    end
% _______________________________________________________________
  %Llena la celda diagonal de A (Con el minimo común multiplo)
  for i=1:p
    aux=An{i,1};
    for j=2:m
        aux=conv(aux,An{i,j});
    end
    A{i,i}=aux;
  end
%________________________________________________________________  
  %Llena la celda de B utilizando el minimo comun multiplo (A)
  for i=1:p
    for j=1:m
        aux=Bn{i,j};
        for k=1:m
            if k~=j
                aux=conv(aux,An{i,k});
            end
        end
        B{i,j}=aux;
    end
  end
end