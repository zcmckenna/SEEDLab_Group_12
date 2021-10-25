
function[] = test_animation(time, phi, xposition, yposition)
%instructions are in assignment 2 for localization
%1. Use test code in Arduino main loop to print out time, angular position, x and y
%position
%2. Type "results = []" into command window to make spreadsheet for
%variable. Type openvar('results') to open spreadsheet
%3. Copy and paste printed data from Arduino serial monitor into
%spreadsheet.
%4. Type "test_animation(results(:,1), results(:,2), results(:,3), results(:,4))" 
% into command window to run animation
%5. May need to change axis to scale



r_width = 0.074;
r_length = 0.1;

len_a = r_length*0.5;
len_b = r_length*0.25;
len_c = r_length*0;
widA = r_width*0.4;
widB = r_width*0.45;
widC = r_width*0.55;
widD = r_width*0.3;
v = [-len_a -len_a  len_c len_c  -len_b -len_b len_b len_b  len_c  len_c  len_a  len_a  len_c  len_c  -len_b -len_b  len_b  len_b len_c  len_c; -widA  widA widA widB widB  widC widC widB widB widA widD -widD -widA -widB -widB -widC -widC -widB -widB -widA];
 

figure 
for i = 1:length(time)
    T = [cos(phi(i)) -sin(phi(i));sin(phi(i)) cos(phi(i))];
    pos  = [xposition(i);yposition(i)];
    v_c = T*v+pos+ones(1,20);
    fill(v_c(1,:),v_c(2,:),'r')
    axis([-2 5 -2 5])
    if i<length(time),pause(time(i+1)-time(i))
    end
end