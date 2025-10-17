
function fkine(numJoints)
    %numJoints = 3;
    
    UI = 1;
    
    %Have to setup array with syms otherwise MATLAB will complain about using
    %numeric vs variables
    A_Mats = sym(zeros(4,4,numJoints));
    
    if UI == 1
        [DH, A_Mats] = UserVarInput(numJoints);
    else
        %If you want to hard set it
        %Matrix(Row, Column)
        
        for index = 1:numJoints
    
            a = DH(index,1);
            d = DH(index,2);
            alpha = DH(index,3);
            theta = DH(index,4);
        end
        % Build the A_i matrix
        A_Mats(:,:,index) = createMatrix(a,d,alpha,theta);
    end
    
    % Displays the A Matrices with pi/180 substituted with 1
    subs(A_Mats(:,:,:),pi/180,1)

    % Multiply all transformation matrices
    Trans = eye(4); 
    for index = 1:numJoints
        Trans = Trans * A_Mats(:,:,index);
    end
    
    sympref('AbbreviateOutput', false);
    Trans = simplify(Trans,"Steps",400);
    %This removes the pi/180 that appears when you just use the simplify
    %function to display
    Trans = subs(Trans,pi/180,1)
    vars = symvar(Trans);
    for index = 1:length(vars)
        vars_str = vars(index);
        if startsWith(string(vars_str), "l")
            
        else
            Trans = subs(Trans,{vars(index)},{input(string(vars_str) + ": ")})
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

%This function isn't necessary as i've hardcoded it. I'll leave this here
%incase I want to repurpose it though.
function [DH, A_Mats] = UserVarInput(numJoints)
    DH = sym(zeros(numJoints,4));
    for index = 1:numJoints
        
        % Ask user for inputs
        a_str = input("Enter a value (number or variable): ", 's');
        d_str = input("Enter d value (number or variable): ", 's');
        alpha_str = input("Enter alpha value: ",'s');
        theta_str = input("Enter theta value: ", 's');
        
        % Convert a
        a_val = str2double(a_str);
        if isnan(a_val)   % not numeric → treat as variable
            DH(index,1) = sym(a_str);
            %Unknowns(end+1) = a_str;
        else
            DH(index,1) = a_val;
        end
        
        % Convert d
        d_val = str2double(d_str);
        if isnan(d_val)
            DH(index,2) = sym(d_str);
            %Unknowns(end+1) = d_str;
        else
            DH(index,2) = d_val;
        end
    
        % Convert alpha
        alpha_val = str2double(alpha_str);
        if isnan(alpha_val)   % not numeric → treat as variable
            DH(index,3) = sym(alpha_str);
            %Unknowns(end+1) = alpha_str;
        else
            DH(index,3) = alpha_val;
        end
    
        % Convert theta
        
        theta_val = str2double(theta_str);
        if isnan(theta_val)   % not numeric → treat as variable
            DH(index,4) = sym(theta_str);
            %Unknowns(end+1) = theta_str;
        else
            DH(index,4) = theta_val;
        end

        % Build the A_i matrix
        A_Mats(:,:,index) = createMatrix(DH(index,1),DH(index,2),DH(index,3),DH(index,4));
    
    end

end