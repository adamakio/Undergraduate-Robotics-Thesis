% Non convex projection
function proj_qdot = projection(qdot)
proj_qdot = zeros("like", qdot);
for i = 1:length(qdot)
    if abs(qdot(i)) <= 0.2
        proj_qdot(i) = qdot(i);
    elseif abs(qdot(i)) <= 0.9
        proj_qdot(i) = 0.2*qdot(i)/abs(qdot(i));    
    else
        proj_qdot(i) = 2.0*qdot(i)/abs(qdot(i));
    end
end
end