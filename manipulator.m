classdef manipulator

    properties
        DH
        A_Mats
        Trans
        numJoints
    end

    methods
    
        function self = manipulator(DH)
            
            self.DH = DH;
            self.numJoints = size(self.DH,1);
        
        end


        function Trans = fkine(self,q,l)
            arguments
                self
                q = []
                l = []
            end

            numJoints = self.numJoints;
            %Have to setup array with syms otherwise MATLAB will complain about using
            %numeric vs variables
            A_Mats = sym(zeros(4,4,numJoints));
            

                
            for index = 1:numJoints
        
                a = self.DH(index,1);
                d = self.DH(index,2);
                alpha = self.DH(index,3);
                theta = self.DH(index,4);

                % Build the A_i matrix
                A_Mats(:,:,index) = createMatrix(a,d,alpha,theta);
            end
            
            
            % pi/180 substituted with 1
            self.A_Mats = subs(A_Mats(:,:,:),pi/180,1);
            
        
            % Multiply all transformation matrices
            Trans = eye(4); 
            for index = 1:numJoints
                Trans = Trans * A_Mats(:,:,index);
            end
            
            sympref('AbbreviateOutput', false);
            Trans = simplify(Trans,"Steps",400);
            %This removes the pi/180 that appears when you just use the simplify
            %function to display
            Trans = subs(Trans,pi/180,1);
            self.Trans = Trans;
            
            %If
            if ~isempty(q)                
                vars = symvar(Trans);
                q_index = 1;
                for index = 1:length(vars)
                    if ~startsWith(string(vars(index)), "l")
                        Trans = subs(Trans,vars(index),q(q_index));
                        q_index = q_index + 1;
                    end
                end
            end
        end

        function q = ikine(self)
            %q = zeroes(self.)

        end

        function J = Jacobian(self)
            

        end
    end
end

%General form of the homgeneous transformation matrix
function A = createMatrix(a,d,alpha,theta)
    A = [cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha)  a*cosd(theta);
         sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha)  a*sind(theta);
         0           sind(alpha)             cosd(alpha)             d;
         0           0                      0                      1];
end