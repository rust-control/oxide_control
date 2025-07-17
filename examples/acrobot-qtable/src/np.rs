pub fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    let step = (end - start) / (n - 1) as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

pub fn digitize(value: f64, bins: &[f64]) -> usize {
    if !bins.is_sorted() {
        panic!("`bins` must be sorted in ascending order");
    }
    if bins.is_empty() {
        return 0;
    }
    for (i, &bin) in bins.iter().enumerate() {
        if value < bin {
            return i;
        }
    }
    bins.len() - 1 // Return the last bin if value is greater than all bins
}

pub fn random(start: f64, end: f64) -> f64 {
    rand::Rng::random_range(&mut rand::rng(), start..end)
}
