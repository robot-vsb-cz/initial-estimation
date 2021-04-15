function [outputArg1] = makeCorkeRobot(x,nJoints)
%   Create a 3-7 DOFs robot using Corke's toolbox, based on input DH and number of Joints


switch nJoints
    case 3
        outputArg1 = SerialLink( [
            Revolute('d', x(4), 'a', x(1), 'alpha', x(7))
            Revolute('d', x(5), 'a', x(2), 'alpha', x(8))
            Revolute('d', x(6), 'a', x(3), 'alpha', x(9))], 'name', '3R');
        outputArg1.base = [x(10) x(11) x(12)];
        
    case 4
        outputArg1 = SerialLink( [
            Revolute('d', x(5), 'a', x(1), 'alpha', x(9))
            Revolute('d', x(6), 'a', x(2), 'alpha', x(10))
            Revolute('d', x(7), 'a', x(3), 'alpha', x(11))
            Revolute('d', x(8), 'a', x(4), 'alpha', x(12))], 'name', '4R');
        outputArg1.base = [x(13) x(14) x(15)];
        
    case 5
        outputArg1 = SerialLink( [
            Revolute('d', x(6), 'a', x(1), 'alpha', x(11))
            Revolute('d', x(7), 'a', x(2), 'alpha', x(12))
            Revolute('d', x(8), 'a', x(3), 'alpha', x(13))
            Revolute('d', x(9), 'a', x(4), 'alpha', x(14))
            Revolute('d', x(10), 'a', x(5), 'alpha', x(15))], 'name', '5R');
        outputArg1.base = [x(16) x(17) x(18)];
        
    case 6
        outputArg1 = SerialLink( [
            Revolute('d', x(7), 'a', x(1), 'alpha', x(13))
            Revolute('d', x(8), 'a', x(2), 'alpha', x(14))
            Revolute('d', x(9), 'a', x(3), 'alpha', x(15))
            Revolute('d', x(10), 'a', x(4), 'alpha', x(16))
            Revolute('d', x(11), 'a', x(5), 'alpha', x(17))
            Revolute('d', x(12), 'a', x(6), 'alpha', x(18))], 'name', '6R');
        outputArg1.base = [x(19) x(20) x(21)];
        
    case 7
        outputArg1 = SerialLink( [
            Revolute('d', x(8), 'a', x(1), 'alpha', x(15))
            Revolute('d', x(9), 'a', x(2), 'alpha', x(16))
            Revolute('d', x(10), 'a', x(3), 'alpha', x(17))
            Revolute('d', x(11), 'a', x(4), 'alpha', x(18))
            Revolute('d', x(12), 'a', x(5), 'alpha', x(19))
            Revolute('d', x(13), 'a', x(6), 'alpha', x(20))
            Revolute('d', x(14), 'a', x(7), 'alpha', x(21))], 'name', '7R');
        outputArg1.base = [x(22) x(23) x(24)];
        
end

end

