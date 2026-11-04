# Froggy production and development

Read viewer.json to identify this checkout. Current production is V12.5 in
~/cad/froggy-cad (master). Active CAD work belongs in ~/cad/froggy-cad-wt/dev
(dev). Preserve PROD geometry and outputs until Sawyer requests a promotion.
Use bin/view to open the clearly labeled viewer for the current checkout.
Build in DEV with bin/cad build froggy_v12_5; each viewer reads its own out/.
Do not share or symlink PROD and DEV output directories. Historical v12.5 and
other worktrees are not the active development checkout.
