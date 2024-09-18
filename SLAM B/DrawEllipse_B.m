function DrawEllipse_B(Zest,Ptotal,p,istant,Landmarks)
   
if nargin==5
    for q=1:(size(Zest,2)-3)/2
        mu=[Zest(istant,2+2*q);Zest(istant,3+2*q)];
        Pel=reshape(Ptotal(end,:),[],size(Zest,2))';
        Sigma= Pel(2+2*q:3+2*q,2+2*q:3+2*q);
        s = -2 * log(1 - p);
        [V, D] = eig(Sigma * s);
        t = linspace(0, 2 * pi);
        a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
        if Zest(istant,2+2*q)==0
           plot(a(1, :) + mu(1), a(2, :) + mu(2),"Color","white",'LineWidth',1.0);
           %scatter(Zest(istant,2+2*q),Zest(istant,3+2*q),'b','+', 'LineWidth', 0.8)
        else
           plot(a(1, :) + mu(1), a(2, :) + mu(2),"Color","#77AC30",'LineWidth',1.0);
           scatter(Zest(istant,2+2*q),Zest(istant,3+2*q),'b','+', 'LineWidth', 0.8)
        end

        hold on     
    end
   else
    mu=[Zest(1,1);Zest(1,2)];
    Pel=reshape(Ptotal(1,:),[],size(Zest,2))';
    Sigma= Pel(2+2*1:3+2*1,2+2*1:3+2*1);
    s = -2 * log(1 - p);
    [V, D] = eig(Sigma * s);
    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    plot(a(1, :) + mu(1), a(2, :) + mu(2),'r','LineWidth',1.0);
    hold on
    
end