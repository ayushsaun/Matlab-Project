%% Mutate function

function q=Mutate(p) 
    
    x=[p.theta1 , p.theta2 , p.theta3 , p.theta4 , p.theta5 , p.theta6];
    nvar=numel(x);
    randrand=randperm(nvar);
    j1=randrand(1);
    j2=randrand(2);
    
    n1j=x(j1);
    n2j=x(j2);
    x(j1)=n2j;
    x(j2)=n1j;
   
    q.theta1 = x(1);
    q.theta2 = x(2);
    q.theta3 = x(3);
    q.theta4 = x(4);
    q.theta5 = x(5);
    q.theta6 = x(6);
    

end