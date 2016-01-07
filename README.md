# space-colonization-rs

[Space-Colonization Algorithm][1] in Rust.

## Screenshot

![Space Colonization Algorithm](/screenshot.png?raw=true "Space Colonization")

## Usage

```sh
cargo run --example simulate --release -- \
    --radius 0.1 \
    --num-points 2000 \
    --kill-distance 0.03 \
    --move-distance 0.003 \
    --num-roots 20 \
    --save-every 100 \
    --max-iter 300
```

[1]: http://algorithmicbotany.org/papers/colonization.egwnp2007.large.pdf
