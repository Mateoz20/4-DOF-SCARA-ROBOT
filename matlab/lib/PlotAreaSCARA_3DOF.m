function PlotAreaSCARA_3DOF(L,fig)
figure(fig); 
hold on; grid on; axis equal
l1 = L(1); l2 = L(2); l3 = L(3);

d1 = l1 + l2 + l3;        % raggio massimo teorico
d2 = abs(l1 - (l2 + l3)); % raggio minimo (molto conservativo)

nn = 50;
for i = 0:nn
    fi = 2*pi/nn*i;
    xa = d1*cos(fi); ya = d1*sin(fi);
    xb = d2*cos(fi); yb = d2*sin(fi);
    if i ~= 0
        plot([xaold xa],[yaold ya],'LineWidth',2,'color','green');
        plot([xbold xb],[ybold yb],'LineWidth',2,'color','green');
    end
    xaold = xa; yaold = ya;
    xbold = xb; ybold = yb;
end
hold off
end