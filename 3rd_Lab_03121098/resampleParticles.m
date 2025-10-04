% Resampling function
function indices = resampleParticles(weights, N_particles)
    cdf = cumsum(weights);
    indices = zeros(1, N_particles);
    for i = 1:N_particles
        sample = rand;
        indices(i) = find(cdf >= sample, 1);
    end
end