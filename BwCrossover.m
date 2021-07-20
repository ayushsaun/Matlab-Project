%% crosspop function

function crosspop=BwCrossover(crosspop,pop,Dim,nCross,npop,nCannibalism,xd,puma)

    ik.theta1 = [];
    ik.theta2 = [];
    ik.theta3 = [];
    ik.theta4 = [];
    ik.theta5 = [];
    ik.theta6 = [];
    ik.fk = [];
    ik.fitness = [];

    a=repmat(ik,npop,1);

    indexno=randperm(nCross);
    
    for k=1:2:nCross
        
        %%  parents' choosing
        r1=indexno(k);
        r2=indexno(k+1);
        
        p1=pop(r1);
        p2=pop(r2);
        
        %% sexual Cannibalism

        if pop(r1).fitness < pop(r2).fitness
            a(1) = pop(r1);
        else
            a(1) = pop(r2);
        end
        
        %% Reproduction
        
        for i=1:2:Dim
        
            x1=[p1.theta1 p1.theta2];
            x2=[p2.theta1 p2.theta2];
            
            x3=[p1.theta3 p1.theta4];
            x4=[p2.theta3 p2.theta4];
            
            x5=[p1.theta5 p1.theta6];
            x6=[p2.theta5 p2.theta6];
            
            alpha=rand(size(x1));
            
            y1=alpha.*x1+(1-alpha).*x2;
            y2=alpha.*x2+(1-alpha).*x1;
            
            y3=alpha.*x3+(1-alpha).*x4;
            y4=alpha.*x4+(1-alpha).*x3;
            
            y5=alpha.*x5+(1-alpha).*x6;
            y6=alpha.*x6+(1-alpha).*x5;
            
            a(i+1).theta1=y1(1);
            a(i+1).theta2=y1(2);
            a(i+1).theta3=y3(1);
            a(i+1).theta4=y3(2);
            a(i+1).theta5=y5(1);
            a(i+1).theta6=y5(2);
            
            a(i+2).theta1=y2(1);
            a(i+2).theta2=y2(2);
            a(i+2).theta3=y4(1);
            a(i+2).theta4=y4(2);
            a(i+2).theta5=y6(1);
            a(i+2).theta6=y6(2);
            
            a(i+1).fk=forward_kinematics([a(i+1).theta1 a(i+1).theta2 a(i+1).theta3 a(i+1).theta4 a(i+1).theta5 a(i+1).theta6],puma);
            a(i+2).fk=forward_kinematics([a(i+2).theta1 a(i+2).theta2 a(i+2).theta3 a(i+2).theta4 a(i+2).theta5 a(i+2).theta6],puma);
             
            a(i+1).fitness = abs(xd(1,:) - a(i+1).fk);
            a(i+2).fitness = abs(xd(1,:) - a(i+2).fk);
            
        end
        
        for i = 1:Dim
            Fitness(i,:) = a(i).fitness;
        end
         
        [Fitness, order] = sort(Fitness);
        a = a(order);
        worstValue = [a(end).theta1 , a(end).theta2 , a(end).theta3 , a(end).theta4 , a(end).theta5 , a(end).theta6];
        bestValue = [a(1).theta1 , a(1).theta2 , a(1).theta3 , a(1).theta4 , a(1).theta5 , a(1).theta6];
        
        %% Sibling Cannibalism
        
             if Dim>2
                 for l=0:nCannibalism
                     crosspop(k+l)=a(l+1);
                 end
             elseif Dim==2
                 for l=0:nCannibalism+1
                     crosspop(k+l)=a(l+1);
                 end
             elseif Dim==1
                 for l=0:nCannibalism+2
                     crosspop(k+l)=a(l+1);
                 end
             end
        
    end
     
end
 