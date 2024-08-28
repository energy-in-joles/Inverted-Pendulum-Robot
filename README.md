# Inverted-Pendulum-Robot (Furuta Pendulum)

## Introduction

A Furuta Pendulum is a control problem which involves a driven arm (free-rotating in the vertical plane) attached to a horizontal rotating plane. The goal of the robot is to balance the arm in the vertical position. This is a more advanced adaptation of the classic [cartpole problem](https://gymnasium.farama.org/environments/classic_control/cart_pole/), where it includes swing-up (swinging the arm to the vertical position in the first place).

## Hardware Implementation

The following parts were procured for the robot assembly:
1. [Nema 17 Stepper Motor 2.8V](https://thepihut.com/products/nema-17-stepper-motor-42mm-x-40mm?srsltid=AfmBOoo_bepqFz6w8FNt053Pt6Cj5KEEfmS52Y3k3W-_BQsMm00a4_xF)
2. [600 PPR Photoeletric Incremental Rotary Encoder](https://www.aliexpress.com/item/1005005071771659.html?spm=a2g0n.productlist.0.0.200d584eQcWS85&browser_id=6f87ffb9137e41d19d24bdc762d160ff&aff_platform=msite&m_page_id=fffbeddbdcdff1904d0c6081b2ff06e10e0b28dc16&pdp_npi=4%40dis%21GBP%216.11%214.28%21%21%217.50%215.25%21%402103868d17192795751537292e66bb%2112000031526886199%21sea%21UK%210%21AB&algo_pvid=7a7e320b-96ee-4f7b-90cf-3c9826d6bb84&_universallink=1&m_page_id=fffbeddbdcdff1904d0c6081b2ff06e10e0b28dc16)
3. [Motor Driver Board](https://www.aliexpress.com/item/1005003166428228.html?src=google&src=google&albch=shopping&acnt=494-037-6276&slnk=&plac=&mtctp=&albbt=Google_7_shopping&gclsrc=aw.ds&albagn=888888&isSmbAutoCall=false&needSmbHouyi=false&src=google&albch=shopping&acnt=494-037-6276&slnk=&plac=&mtctp=&albbt=Google_7_shopping&gclsrc=aw.ds&albagn=888888&ds_e_adid=&ds_e_matchtype=&ds_e_device=c&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=en1005003166428228&ds_e_product_merchant_id=477639041&ds_e_product_country=GB&ds_e_product_language=en&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=17866112291&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&aff_fcid=83dcffd2eb2741f7ad2c1d928aab8221-1719349569575-02419-UneMJZVf&aff_fsk=UneMJZVf&aff_platform=aaf&sk=UneMJZVf&aff_trace_key=83dcffd2eb2741f7ad2c1d928aab8221-1719349569575-02419-UneMJZVf&terminal_id=05d320940694439db396902b2f4edb24&afSmartRedirect=y)
4. [12V 2A Power Supply Plug Charger](https://www.lazada.sg/products/msrc-safety-5v-12v-1a-2a-3a-100-240v-mains-transformer-acdc-adapter-uk-plug-charger-power-supply-i1875876088-s10002441740.html?c=&channelLpJumpArgs=&clickTrackInfo=query%253A12v%252B2a%252Bpower%252Badapter%253Bnid%253A1875876088%253Bsrc%253ALazadaMainSrp%253Brn%253A5c7f04da87ed32bc1c4a6d6fdd44f340%253Bregion%253Asg%253Bsku%253A1875876088_SGAMZ%253Bprice%253A2.69%253Bclient%253Adesktop%253Bsupplier_id%253A1000182085%253Bbiz_source%253Ah5_internal%253Bslot%253A2%253Butlog_bucket_id%253A470687%253Basc_category_id%253A81%253Bitem_id%253A1875876088%253Bsku_id%253A10002441740%253Bshop_id%253A272449&fastshipping=0&freeshipping=1&fs_ab=2&fuse_fs=&lang=en&location=China&price=2.69&priceCompare=skuId%3A10002441740%3Bsource%3Alazada-search-voucher%3Bsn%3A5c7f04da87ed32bc1c4a6d6fdd44f340%3BunionTrace%3A4f85b09517193492513388726e%3BoriginPrice%3A269%3BvoucherPrice%3A269%3BdisplayPrice%3A269%3BsinglePromotionId%3A91471144322198%3BsingleToolCode%3ApromPrice%3BvoucherPricePlugin%3A1%3BbuyerId%3A0%3Btimestamp%3A1719349251886&ratingscore=4.899408284023669&request_id=5c7f04da87ed32bc1c4a6d6fdd44f340&review=169&sale=1306&search=1&source=search&spm=a2o42.searchlist.list.2&stock=1)
5. DRV8825 Stepper Motor Driver
6. Arduino Nano

The pendulum and encoder housing were designed in Fusion 360 and produced with FDM printing. The `.step` and `.f3d` files can be found in `/robot/CAD`.

The pendulum was attached to the encoder with a rigid shaft coupler. The encoder housing was attached to the stepper motor with a modified M6 bolt in a T-nut fitted in another rigid shafter coupler.

The Arduino communicates with the Python script with bi-directional serial communication, where it:
1. Reads the acceleration input for the motor from the PC and implements it.
2. Writes the motor position and pendulum position data for generating the observation space for the model.
3. Implements reset functions for resetting the motor position and encoder value.

## Pendulum Env ##

Follows the standard Gymnasium class format. Defined below is the observation space, action space, and reward function used for the experiment:

- $\theta$: angular position of pendulum. 0 at the top and normalised between $[-\pi,\pi]$.
- $\dot{\theta}$: angular velocity of pendulum. Experimentally bounded between $[-10,10]$, then normalised to between $[-2, 2]$.
- $\alpha$: motor position (measured in steps instead of angle). Steps range physically limited to 90 degrees left and right or $[-200, 200]$ step range. However, observation space spans further between $[-300, 300]$ to account for the motor exceeding the limit slightly. The range is then normalised between $[-3,3]$.
- $\dot{\alpha}$: motor velocity (steps per second). Experimentally bounded between $[-4, 4]$, then normalised to between $[-1, 1]$
- $\ddot{\alpha}$: motor acceleration (steps per second squared). Control input into the system. Bounded between $[-20000,20000]$ and normalised between $[-2, 2]$.

Note that all values are continuous.

### Observation Space ###
$[\cos{\theta}, \sin{\theta}, \dot{\theta}, \alpha, \dot{\alpha}]$

`ndarray` of size (5,) containing 5 continuous observation values. Using $\cos$ and $\sin$ values of $\theta$ experimentally showed better convergence rates than just $\theta$.

### Action Space ###
$[\ddot{\alpha}]$

`ndarray` of size (1,) containing motor acceleration value. Continuous action space.

### Reward Function ###
$\gamma-(\theta^2+C_2\times\dot{\theta}^2+C_3\times\alpha^2+C_4\times\dot{\alpha}^2+C_5\times\ddot{\alpha}^2)$

$\gamma$: reward offset value (offset) to ensure that reward is always positive. 

If reward was $-\inf$ to $0$, episodes with early termination would generate a higher reward (because they would accumulate smaller negative reward), resulting in a faulty reward system. Hence, the offset aims to prevent this issue. Constants $C$ are used to adjust the reward function weightage and are defined in the `/conf/config.yaml`.

## Usage

### Configuration

The script provides a high level of flexibility for training and evaluating the model. Configurations are stored in the `conf` folder, organised in the following manner:
1. `config.yaml`
    - model selection (PPO or SAC)
    - mode selection (train or eval)
    - serial communication configuration between PC and arduino
    - action and observation space configuration
    - reward function weights
    - toggle logging with tensorboard
2. `/mode/train.yaml`:
    - new model file configuration 
    - device config (cuda or cpu)
    - training total timesteps and episode length
3. `/mode/eval.yaml`:
    - device config (cuda or cpu)
    - episode length set to -1 for infinite episode length
4. `/model` (`PPO.yaml` and  `SAC.yaml`) config files:
    - model weights

### Install and Run

1. To run the code, first build dependencies in the root directory with:
```pip install -e .```

2. Upload the Arduino code in `/robot/arduino/main/main.ino` to the Arduino board.

3. Connect the robot to the PC and run the script, remembering to set the mode to `train`:
```python src/main.py```

4. The trained model will then be stored in `/model`, where it can be trained further or evaluated.

## Credits
We largely referred to the following resources for guidance:
1. Armandpl's [video](https://www.youtube.com/watch?v=Y6FVBbqjR40) and [repository](https://github.com/Armandpl/furuta/tree/master?tab=readme-ov-file#mlops) on building a similar project as us.
2. Inspiration for this project was taken from Quanser Qube design. Reward function adapted from Quanser's [code](https://git.ias.informatik.tu-darmstadt.de/quanser/clients/-/tree/master/quanser_robots/qube).
3. Stable Baseline's [guide](https://stable-baselines.readthedocs.io/en/master/guide/custom_env.html) on custom environment creation.
4. [Farama's guide on handling termination vs truncated scenarios](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/) when designing our environment.