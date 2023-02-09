function YY=Backstepping_control_reach_90degrees()
    clear;
    clear all;
    clc;
    
    hat=@(y)[0,-y(3),y(2);y(3),0,-y(1);-y(2),y(1),0];
    
    global p R j n m v u q w vs us vt ut qt wt vst ust vh uh vsh ush qh wh nLL mLL x y z X Y Z  %Make vars available in whole program
    global  Tt1 Tt2 rt1 rt2 ns e_dot e C
    
    %Parameters
    L = 0.5;                       %Length (before strain)
    N = 50;                        %Spatial resolution
    E = 207e9;                     %Young's modulus
    r = 0.001;                     %Cross-section radius
    rt1 = [0.02;0;0];
    rt2 = [-0.02;0;0];
    rho = 8000;                    %Density
    g = [-9.81;0;0];               %Gravity
    Bse = zeros(3);                %Material damping coefficients - shear and extension
    Bbt = 1e-2*eye(3);             %Material damping coefficients - bending and torsion
    C = 0.1*eye(3);
    dt = 0.04;                    %Time step
    alpha = -0.2;                  %BDF-alpha parameter
    STEPS = 202;                   %Number of timesteps to completion
    vstar = @(s)[0;0;1];        %Value of v when static and absent loading
    ustar = @(s)[0;0;0];           %Precurvature
    vsstar = @(s)[0;0;0];
    usstar = @(s)[0;0;0];
    
    %Boundary Conditions
    for i = 1 : STEPS
        p{i,1} = [0;0;0];          %Clamped base
        R{i,1} = eye(3);
        q{i,1} = [0;0;0];
        w{i,1} = [0;0;0];
    end
    
    nL = [0;0;0];                    
    mL = [0;0;0];

    %Dependent Parameter Calculations
    A = pi*r^2;                                 %Cross-sectional area
    J = diag([pi*r^4/4  pi*r^4/4  pi*r^4/2]);   %Inertia
    G = 80e9;                                   %Shear modulus
    Kse = diag([G*A, G*A, E*A]);                %Stiffness matrix - shear and extension
    Kbt = diag([E*J(1,1), E*J(2,2), G*J(3,3)]); %Stiffness matrix - bending and torsion
    ds = L/(N-1);                               %Grid distance (before strain)
    c0 = (1.5 + alpha) / ( dt*(1+alpha) );      %BDF-alpha coefficients
    c1 = -2/dt;
    c2 = (0.5 + alpha) / ( dt*(1+alpha) );
    d1 = alpha / (1+alpha);
    
    %Main Simulation
    i = 1;
    fsolve(@staticIVP, zeros(6,1)); %Solve static BVP w/ shooting method
    applyStaticBDFalpha();
%   visualize1();


    f=1;
    
    for i = 2 : STEPS
        
        
         if i<3
            Tt1=0;
            Tt2=0;
         else
             
            TT=find_tension(); %Backstepping Control
              if TT>0
                 Tt1=TT; 
              else
                 Tt1=0;
              end
         end

        fsolve(@dynamicIVP, [n{i-1,1}; m{i-1,1}]); %Solve semi-discretized PDE w/ shooting
        applyDynamicBDFalpha();
        visualize();
          
        if i<3
            e=0;
            e_dot=0;
            TT=0;
        end
        
YY(f,:)=[(i-2)*dt,TT,p{i-1,N}(1),p{i-1,N}(3),e(1),e_dot(1),Tt1];
p{i-1,N}(1)
f=f+1;

    end

    
    
    %Function Definitions
    function TT= find_tension

        t=(i-2)*dt;

        P_desire= [0.25*((1-exp(-2*t)));0;0] ;
        Pd_desire= [0.25*(2*exp(-2*t));0;0];
        Pdd_desire= [0.25*(-4*exp(-2*t));0;0] ;
        
        p_dot1= R{i-1,N-1}*(hat(u{i-1,N-1})*rt1+ v{i-1,N-1});
        p_ddot1 = R{i-1,N-1}*(hat(u{i-1,N-1})*(hat(u{i-1,N-1})*rt1+ v{i-1,N-1})+hat(us{i-1,N-1})*rt1+ vs{i-1,N-1}); 
        
        alph1 = (hat(p_dot1)*hat(p_dot1)*p_ddot1)/(norm(p_dot1)^3);
        alph11 = (p_dot1)/(norm(p_dot1));

        alphM=alph1+alph11;

        ac= (ns + rho*A*g - R{i-1,N-1}*C*q{i-1,N-1}.*abs(q{i-1,N-1}))/(rho*A);
        bc=-(alphM)/(rho*A);
        
        X_end= p{i-1,N};
        Xd_end=(R{i-1,N}*q{i-1,N});
        
        alpa1=64.7; 
        alpa2=10;
        
        Z1= P_desire-X_end;
        Z2= Xd_end-Pd_desire-alpa1*Z1;
                
        TT=bc\(Z1-ac-alpa1*(Z2+alpa1*Z1)-alpa2*Z2+Pdd_desire);
        
        if TT>87 
           TT=87; %fsolve can be solved in this condition
        end
        
        e=P_desire-X_end;
        e_dot=Pd_desire-Xd_end;
        
        
    end

    function E = staticIVP(G)
        n{i,1} = G(1:3);
        m{i,1} = G(4:6);

        %Euler's method
        for j = 1 : N-1
            [ps, Rs, ns, ms, vs{i,j}, us{i,j} ,v{i,j}, u{i,j}] = staticODE(p{i,j},R{i,j},n{i,j},m{i,j});
            p{i,j+1} = p{i,j} + ds*ps;
            R{i,j+1} = R{i,j} + ds*Rs;
            n{i,j+1} = n{i,j} + ds*ns;
            m{i,j+1} = m{i,j} + ds*ms;
        end
        E = [ n{i,N} - nL;  m{i,N} - mL ];
    end

    function [ps, Rs, ns, ms, vs, us, v, u] = staticODE(p,R,n,m)
        v = Kse\R'*n + vstar(ds*(j-1));
        u = Kbt\R'*m + ustar(ds*(j-1));
        
        ptsb = hat(u)*rt1+v;
        Tt = 0;
        At = -Tt/norm(ptsb)^3*hat(ptsb)*hat(ptsb);
        B = hat(rt1)*At;
        Gt = -At*hat(rt1);
        H =-B*hat(rt1);
        a = At*(hat(u)*ptsb);
        b = hat(rt1)*a;
        
        fe=0*rho*A*g;
        d = Kse*vsstar(ds*(j-1))-hat(u)*Kse*(v-vstar(ds*(j-1)))-a-R'*fe;
        c = Kbt*usstar(ds*(j-1))-hat(u)*Kbt*(u-ustar(ds*(j-1)))-hat(v)*Kse*(v-vstar(ds*(j-1)))-b;

        
        Mat = [Kse+At,Gt;B,Kbt+H];
        vs = 1/det(Mat)*((Kbt+H)*d-Gt*c);
        us = 1/det(Mat)*(-B*d+(Kse+At)*c);
        
        
        ps = R*v;
        Rs = R*hat(u);
        ns = [0;0;0];
        ns = -fe;
        ms = -hat(ps)*n;
        
    end

    function applyStaticBDFalpha()
        for j = 1 : N-1
            vh{i+1,j} = (c1+c2)*v{i,j};
            uh{i+1,j} = (c1+c2)*u{i,j};
            vsh{i+1,j} = (c1+c2)*vs{i,j};
            ush{i+1,j} = (c1+c2)*us{i,j};
            qh{i+1,j} = [0;0;0];
            wh{i+1,j} = [0;0;0];
            q{i,j} = [0;0;0];
            w{i,j} = [0;0;0];
        end
    end

    function applyDynamicBDFalpha()
        for j = 1 : N-1        
            vh{i+1,j} = c1*v{i,j} + c2*v{i-1,j} + d1*vt{i,j};
            uh{i+1,j} = c1*u{i,j} + c2*u{i-1,j} + d1*ut{i,j};
            vsh{i+1,j} = c1*vs{i,j} + c2*vs{i-1,j} + d1*vst{i,j};
            ush{i+1,j} = c1*us{i,j} + c2*us{i-1,j} + d1*ust{i,j};
            qh{i+1,j} = c1*q{i,j} + c2*q{i-1,j} + d1*qt{i,j};
            wh{i+1,j} = c1*w{i,j} + c2*w{i-1,j} + d1*wt{i,j};
        end
    end

    function E = dynamicIVP(G)
        n{i,1} = G(1:3);
        m{i,1} = G(4:6);

        %Euler's method
        for j = 1 : N-1
            [ps, Rs, ns, ms, qs, ws, vs{i,j}, us{i,j},...
                 v{i,j}, u{i,j}, vt{i,j}, ut{i,j},...
                 qt{i,j}, wt{i,j},vst{i,j}, ust{i,j}] = dynamicODE(p{i,j},R{i,j},n{i,j},m{i,j},q{i,j},w{i,j});
            p{i,j+1} = p{i,j} + ds*ps;
            R{i,j+1} = R{i,j} + ds*Rs;
            n{i,j+1} = n{i,j} + ds*ns;
            m{i,j+1} = m{i,j} + ds*ms;
            q{i,j+1} = q{i,j} + ds*qs;
            w{i,j+1} = w{i,j} + ds*ws;

        end
        E = [n{i,N} - nLL ;  m{i,N} - mLL];

    end



    function [ps,Rs,ns,ms,qs,ws,vs,us,v,u,vt,ut,qt,wt,vst,ust] = dynamicODE(p,R,n,m,q,w)
        v = (Kse + c0*Bse)\(R'*n + Kse*vstar(ds*(j-1)) - Bse*vh{i,j});
        u = (Kbt + c0*Bbt)\(R'*m + Kbt*ustar(ds*(j-1)) - Bbt*uh{i,j});
        vt = c0*v + vh{i,j};
        ut = c0*u + uh{i,j};
        qt = c0*q + qh{i,j};
        wt = c0*w + wh{i,j};
                
        ptsb1 = hat(u)*rt1+v;
        At1 = (-Tt1*hat(ptsb1)*hat(ptsb1))/(norm(ptsb1)^3);
        Bt1= hat(rt1)*At1;
        Gt1 = -At1*hat(rt1);
        H1 =-Bt1*hat(rt1);
        a1 = At1*(hat(u)*ptsb1);
        b1 = hat(rt1)*a1;
        
        
        ptsb2 = hat(u)*rt2+v;
        At2 = (-Tt2*hat(ptsb2)*hat(ptsb2))/(norm(ptsb2)^3);
        Bt2= hat(rt2)*At2;
        Gt2 = -At2*hat(rt2);
        H2 =-Bt2*hat(rt2);
        a2 = At2*(hat(u)*ptsb2);
        b2 = hat(rt2)*a2;
        
        a = a1 + a2;
        b = b1 + b2;
        At = At1 + At2;
        Bt = Bt1 + Bt2;
        Gt = Gt1 + Gt2;
        H = H1 + H2;
               
        
        LamdaN = -a + rho*A*(hat(w)*q + qt)+ C*q.*abs(q)- R'*rho*A*g; %OK
        LamdaM = -b + rho*(hat(w)*J*w + J*wt) - hat(v)*(Kse*(v-vstar(ds*(j-1)))+Bse*vt); %OK
        GammaN = hat(u)*(Kse*(v-vstar(ds*(j-1)))+Bse*vt)-Kse*vsstar(ds*(j-1))+Bse*vsh{i,j};
        GammaM = hat(u)*(Kbt*(u-ustar(ds*(j-1)))+Bbt*ut)-Kbt*usstar(ds*(j-1))+Bbt*ush{i,j};

        
        
        Mat = [Kse+c0*Bse+At,Gt;Bt,Kbt+c0*Bbt+H];
        
        vs = 1/det(Mat)*((Kbt+c0*Bbt+H)*(-GammaN+LamdaN)-Gt*(-GammaM+LamdaM));
        us = 1/det(Mat)*(-Bt*(-GammaN+LamdaN)+(Kse+c0*Bse+At)*(-GammaM+LamdaM));

        vst = c0*vs + vsh{i,j};
        ust = c0*us + ush{i,j};
        
        
        
        pts1 = R*hat(u)*rt1+R*v;
        pts2 = R*hat(u)*rt2+R*v;

        nLL = -Tt1*pts1/norm(pts1)-Tt2*pts2/norm(pts2);
        mLL = -Tt1*hat(R*rt1)*pts1/norm(pts1)-Tt2*hat(R*rt2)*pts2/norm(pts2);
        
        
        ps = R*v;
        Rs = R*hat(u);
           if i>=3
             fe = rho*A*g;
           else
             fe = 0*rho*A*g ;
           end
        ns = rho*A*R*(hat(w)*q + qt) -R*(a+At*vs+Gt*us)-fe;
        ms = -R*(b+Bt*vs+H*us)+rho*R*(hat(w)*J*w + J*wt) - cross(ps,n);
        qs = vt - hat(u)*q + hat(w)*v;
        ws = ut - hat(u)*w;
    end

   function visualize1()
        for j = 1 : N,  x(j) = p{i,j}(1);  y(j) = p{i,j}(2); z(j) = p{i,j}(3);   end
        figure (1)
        plot(z,x); axis([-0.05*L 1.1*L  -0.05*L 1.1*L]);

        xlabel('z (m)');  ylabel('x (m)'); 
        hold on; grid on;  drawnow;  pause(0.05);
    end


    function visualize()
        if rem(i,1) == 0; 
        for j = 1 : N,  x(j) = p{i,j}(1);  y(j) = p{i,j}(2); z(j) = p{i,j}(3); 
        end
        figure (1)
        
        plot(z,x); axis([-0.5*L 1.1*L  -(0.1/0.3)*L (0.3/0.3)*L]);
        xlabel('z (m)');  ylabel('x (m)'); 
%       hold on; 
        grid on;  drawnow;  pause(0.05);
        end
    end

        for j = 1 : N,  x(j) = p{i,N}(1);  y(j) = p{i,j}(2); z(j) = p{i,j}(3); 
        end

    figure (2)    
    plot(YY(1:STEPS-1,1),YY(1:STEPS-1,3),'.-');
    grid on
    xlabel('t (s)');  ylabel('x (m)'); title('Tip Displacement - X Component');
    
    figure (3)    
    plot(YY(1:STEPS-1,1),YY(1:STEPS-1,4),'.-');
    grid on
    xlabel('t (s)');  ylabel('z (m)'); title('Tip Displacement - Z Component');

    figure (4)    
    plot(YY(1:STEPS-1,1),YY(1:STEPS-1,7),'.-');
    grid on
    xlabel('t (s)');  ylabel('Tension'); 
    
    figure (5) 
    plot(YY(1:STEPS-1,1),YY(1:STEPS-1,5),'.-');
    grid on
    xlabel('t (s)');  ylabel('e (m)'); 

end

