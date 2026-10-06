# Evidence provenance and review

Primary input: results.zip uploaded by the user on 6 October 2026. It includes 20 Monte Carlo trials absent from MATLAB_Results_For_Review.zip. All 10 shared files were byte-identical across the two archives. The larger results.zip is therefore the complete source for this release.

The user log records MATLAB R2024a (24.1.0.2537033), PCWIN64, 6 October 2026, 21:34:05–21:34:33 for the main suite. Monte Carlo CSV/MAT files were supplied separately within the complete results archive; their timing is not included in that main-suite log.

Review recomputed all eight controller RMS values from raw error arrays, checked paired Monte Carlo differences, checked increasing mission timestamps and zero negative-clearance samples, and extracted summary values directly from supplied MAT data. No MATLAB rerun was performed during report preparation. Earlier assistant-generated Octave results were not substituted for the user's data.

A simulation MP4 and single-frame GIF were supplied later on 6 October 2026. The MP4 is included unchanged; a genuine animated GIF preview was derived from it. Numerical measurements remain based on the original MAT/CSV evidence. See MEDIA.md for media details.

Raw result hashes (SHA-256):

- `controller_comparison.png`: `7eeaa483d7cfaa26e002fa2fb8e7d071d8b7179e67e931d90fe0850612eae6e9`
- `controller_experiments.mat`: `329f5c78cbcb3de27aa31b4b45817d07943f83613f5a069531648733e9761036`
- `controller_metrics.csv`: `941c41a995199c23806f3e60587cb4262f7beb84df75d367fe98bcbcfe7b70e0`
- `live_slam_map.png`: `ce5b7753abf736a6f20e33aa4354ee487b468734d848f6d1330d068ab2b4671a`
- `matlab_run_log.txt`: `b87db877491d8f6580a540d8fb499b2f086f55007faa09ff7621bf94b4b6ce2d`
- `mission.mat`: `f793c2bf5ceaf16839fd28f3263d71e14c141e00e76e020bb54e348cacf22552`
- `mission_analysis.png`: `21db1c4ba24a072b6c9c4d8412500b83a01c599648f336c361d3b1b43fa27f62`
- `mission_report.md`: `89c8f96d22259c9a133fc1819a6d58c811e83bad0bc9ef87cf86c71d941ff08c`
- `monte_carlo.csv`: `3da730da9abaec23449040f0a844d68e7efe3a4edd0dba5048a698294e81dd06`
- `monte_carlo.mat`: `bd75320b22ba15a8b72f8264ed654a8afb5cde9f4ae5ce354c7cca0d41c73a87`
- `payload_singularity.png`: `301271ad7432cfa49c3cb224cbbe6491912de6fc1d2f12296c760eef0f80db64`
- `slam_validation.mat`: `12ee46fed06a2087574808db2948ba915031c68f60c9c6bd4a77dbcbf9763339`
