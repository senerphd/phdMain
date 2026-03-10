function y = saturate(u, umin, umax)
    y = min(max(u, umin), umax);
end