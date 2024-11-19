function T=L_Matriz_06(DH)
    DoF=size(DH,1);
    for i=1:DoF  % antes estaba i=1:DoF
        T_ij(:,:,i)=cambio_ij(DHp,i-1,i);
    end

    T(:,:,1)=T_ij(:,:,1);

    for i=1:DoF-1 % 
        T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1);  %AnlDirMS6R - 6DoF
    end
end