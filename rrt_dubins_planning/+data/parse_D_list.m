function [idx, ...
 x_s, y_s, th_s, ...
 x_f, y_f, th_f, ...
 x_s_c, y_s_c, ...
 x_f_c, y_f_c, ...
 d, ...
 x_s_t, y_s_t, the_0_s_t, ...
 x_f_t, y_f_t, the_0_f_t, ...
 the_0_s_s, the_0_f_f, ...
 C1, C2] = parse_D_list(D)
% D (1 x 22) layout
% -------------------------------------------------------------------------
%  1  : idx                (path type index, e.g., LSL/RSR/...)
%
%  2  : x_s                (start waypoint x)
%  3  : y_s                (start waypoint y)
%  4  : th_s               (start heading angle)
%
%  5  : x_f                (final waypoint x)
%  6  : y_f                (final waypoint y)
%  7  : th_f               (final heading angle)
%
%  8  : x_s_c              (start circle center x)
%  9  : y_s_c              (start circle center y)
%
% 10  : x_f_c              (final circle center x)
% 11  : y_f_c              (final circle center y)
%
% 12  : d                  (total Dubins path distance)
%
% 13  : x_s_t              (start tangent point x)
% 14  : y_s_t              (start tangent point y)
% 15  : the_0_s_t          (start circle → tangent exit angle)
%
% 16  : x_f_t              (final tangent point x)
% 17  : y_f_t              (final tangent point y)
% 18  : the_0_f_t          (final circle → tangent entry angle)
%
% 19  : the_0_s_s          (start circle → start pose angle)
% 20  : the_0_f_f          (final circle → final pose angle)
%
% 21  : C1                 (start turn direction: 'L' or 'R')
% 22  : C2                 (final turn direction: 'L' or 'R')
% -------------------------------------------------------------------------

    idx =  D(1);
    x_s = D(2);
    y_s = D(3);
    th_s = D(4);
    x_f = D(5);
    y_f = D(6);
    th_f = D(7);
    x_s_c = D(8);
    y_s_c = D(9);
    x_f_c = D(10);
    y_f_c = D(11);
    d = D(12);
    x_s_t = D(13);
    y_s_t = D(14);
    the_0_s_t = D(15);
    x_f_t = D(16);
    y_f_t = D(17);
    the_0_f_t  = D(18);
    the_0_s_s  = D(19);
    the_0_f_f = D(20);
    C1 = D(21);
    C2 = D(22);

end