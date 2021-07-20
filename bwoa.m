% BWOA FUNCTION
 
function bestvalue = bwoa(RepNo,Q1,Q2,Q3,Q4,Q5,Q6,Dim,xd,puma,FITNESS,z)
    
    while norm(RepNo, 3) > 0.002
        
        %% Black Widow Search Parametters
        npop = 100;                             % Population Number
        maxItr = 100;                          % max number of iteration
        pc=0.8;                                 % Percent of Crossover 
        nCross=round(pc*npop/2)*2;              % Number of selected Parents
         % pMutation=1-pc;                      % Percent of Mutation
        pMutation=0.4;
        nMutation=round(pMutation*npop);        % Number of Mutants

        pCannibalism=0.5;
        nCannibalism=round(pCannibalism*Dim);

        %% Generating initial population

        ik.theta1 = [];
        ik.theta2 = [];
        ik.theta3 = [];
        ik.theta4 = [];
        ik.theta5 = [];
        ik.theta6 = [];
        ik.fk = [];
        ik.fitness = [];

        pop=repmat(ik,npop,1);
        
        for i = 1:npop
            pop(i).theta1 = (Q1(1) + (Q1(2) - Q1(1)).*rand(1))*(pi/180);
            pop(i).theta2 = (Q2(1) + (Q2(2) - Q2(1)).*rand(1))*(pi/180);
            pop(i).theta3 = (Q3(1) + (Q3(2) - Q3(1)).*rand(1))*(pi/180);
            pop(i).theta4 = (Q4(1) + (Q4(2) - Q4(1)).*rand(1))*(pi/180);
            pop(i).theta5 = (Q5(1) + (Q5(2) - Q5(1)).*rand(1))*(pi/180);
            pop(i).theta6 = (Q6(1) + (Q6(2) - Q6(1)).*rand(1))*(pi/180);
            pop(i).fk = forward_kinematics([pop(i).theta1 , pop(i).theta2 , pop(i).theta3 , pop(i).theta4 , pop(i).theta5 , pop(i).theta6],puma);
            pop(i).fitness = abs(xd(1,:) - pop(i).fk);
        end

        % sorting apond checking how the best value is on robot

        Fitness = zeros(npop , 3);
        for i = 1:npop
            Fitness(i,:) = pop(i).fitness;
        end

        [Fitness, order] = sort(Fitness);
        pop = pop(order);
        worstValue = [pop(end).theta1 , pop(end).theta2 , pop(end).theta3 , pop(end).theta4 , pop(end).theta5 , pop(end).theta6];
        bestValue = [pop(1).theta1 , pop(1).theta2 , pop(1).theta3 , pop(1).theta4 , pop(1).theta5 , pop(1).theta6];
        forward_kinematics(bestValue,puma);

        %% Main Loop


        for it=1:maxItr

            % Crossover Operator- Generating the Pop2 population
            crosspop=repmat(ik,nCross,1);
            crosspop=BwCrossover(crosspop,pop,Dim,nCross,npop,nCannibalism,xd,puma);

            % Mutation- Generating the Pop3 population
            pop3=repmat(ik,nMutation,1);
            randnum=randperm(nCross);
            for k=1:nMutation
               i=randnum(k);     
                q=Mutate(pop(i));
                q.fk = forward_kinematics([q.theta1 , q.theta2 , q.theta3 , q.theta4 , q.theta5 , q.theta6],puma);

                q.fitness = abs(xd(1,:) - q.fk);

                pop3(k).theta1=q.theta1;
                pop3(k).theta2=q.theta2;
                pop3(k).theta3=q.theta3;
                pop3(k).theta4=q.theta4;
                pop3(k).theta5=q.theta5;
                pop3(k).theta6=q.theta6;
                pop3(k).fk=q.fk;
                pop3(k).fitness = q.fitness;

            end


            % Unify the Populations (pop, pop2, pop3)
            [pop]=[crosspop
                    pop3];

            % Sorting the Population based on fitness value
            Fitness = zeros(npop , 3);
            for i = 1:npop
                Fitness(i,:) = pop(i).fitness;
            end

            [Fitness, order] = sort(Fitness);
            pop = pop(order);
            worstValue = [pop(end).theta1 , pop(end).theta2 , pop(end).theta3 , pop(end).theta4 , pop(end).theta5 , pop(end).theta6];
            bestValue = [pop(1).theta1 , pop(1).theta2 , pop(1).theta3 , pop(1).theta4 , pop(1).theta5 , pop(1).theta6]; 

            RepNo = pop(1).fitness;
             
            iteration(z)=z;
            q1(z)=pop(1).theta1;
            q2(z)=pop(1).theta2;
            q3(z)=pop(1).theta3;
            q4(z)=pop(1).theta4;
            q5(z)=pop(1).theta5;
            q6(z)=pop(1).theta6;
            f1(z)=pop(1).fitness(1);
            f2(z)=pop(1).fitness(2);
            f3(z)=pop(1).fitness(3);
            z=z+1;
            
            pop(1)
            
        end

            % this basically checks whether the new developed population is
            % better than previous one or not and based on that it converge 
            % Q1,Q2,Q3,Q4,Q5 and Q6
            if (pop(1).fitness <= FITNESS(1,:)) == [1 1 1]
                
                FITNESS(1,:) = pop(1).fitness;

                Q1 = [pop(1).theta1*(180/pi) - 30 , pop(1).theta1*(180/pi) + 30];
                Q2 = [pop(1).theta2*(180/pi) - 30 , pop(1).theta2*(180/pi) + 30];
                Q3 = [pop(1).theta3*(180/pi) - 30 , pop(1).theta3*(180/pi) + 30];
                Q4 = [pop(1).theta4*(180/pi) - 30 , pop(1).theta4*(180/pi) + 30];
                Q5 = [pop(1).theta5*(180/pi) - 30 , pop(1).theta5*(180/pi) + 30];
                Q6 = [pop(1).theta6*(180/pi) - 30 , pop(1).theta6*(180/pi) + 30];
                
                puma.plot(bestValue)
                
            end

            bestvalue = [pop(1).theta1 , pop(1).theta2 , pop(1).theta3 , pop(1).theta4 , pop(1).theta5 , pop(1).theta6];
            
    end

    %% Plotting final solution
   
    figure(1);
    puma.plot(bestvalue)
    title('Inverse Kinematics');
    
%     figure(2);
%     plot(iteration,q1,iteration,q2,iteration,q3,iteration,q4,iteration,q5,iteration,q6);
%     title('\theta_{1}, \theta_{2}, \theta_{3}, \theta_{4}, \theta_{5} and \theta_{6} VS number of iteration')
%     xlabel('number of iterations')
%     ylabel('\theta (radian)')
%     legend('\theta_{1}','\theta_{2}','\theta_{3}','\theta_{4}','\theta_{5}','\theta_{6}')
%     grid on;
%     
%     figure(5);
%     plot(iteration,f1,iteration,f2,iteration,f3);    
%     title('Fitness VS number of iterations')
%     xlabel('number of iterations')
%     ylabel('Fitness along X,Y and Z axis')
%     legend('Fitness along X axis','Fitness along Y axis','Fitness along Z axis')
%     grid on;

end