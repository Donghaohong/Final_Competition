function idx = systematicResample(weights)
% systematicResample Systematic resampling for normalized particle weights.

weights = weights(:);
weights = weights / max(sum(weights), eps);

N = numel(weights);
edges = cumsum(weights);
edges(end) = 1.0;

r = rand / N;
positions = r + (0:N-1)' / N;

idx = zeros(N, 1);
i = 1;
j = 1;

while i <= N
    if positions(i) <= edges(j)
        idx(i) = j;
        i = i + 1;
    else
        j = j + 1;
    end
end
end
