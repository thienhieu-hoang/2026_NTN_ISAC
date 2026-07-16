Sources: C:\Users\AT30890\OneDrive - ETS\Documents\File-ETS\Courses\Wireless Comm\books

If the slot duration is smaller than the coherence time ($T_{\text{slot}} < T_c$), the small-scale fading coefficients ($g$) of consecutive time slots are highly correlated and change smoothly over time rather than behaving as independent random variables.

The sources you provided contain highly relevant descriptions of this physical behavior and include the mathematical models you mentioned (such as the **Jakes model** and the **Gauss-Markov / AR(1) model**). Below are the exact citations and details from your documents:

### 1. Verification of Fading Correlation over Short Durations

Your sources explicit state that within a timeframe smaller than the coherence time ($T_c$), the time-varying channel is strongly correlated or approximately constant:

* **Document:** `_on_time_varying_channel.pdf`
* **Section:** *1.4.1.5 Global Channel Parameters* (Pages 18–19)
* 
**Details:** The text notes that coherence time ($T_c$) quantifies the duration within which the channel is strongly correlated. Using a Taylor series approximation, it mathematically demonstrates that within durations smaller than $T_c$, the channel stays strongly correlated or approximately constant in a mean-square sense. For a slot or frame duration shorter than $T_c$, this simplifies to a model known as **block flat fading** where the channel coefficient is treated as constant within that frame but smoothly correlated over a larger time scale.


* **Document:** `_ongoing_Proakis, Digital Communications, 5th ED.pdf` (and the matching `John_G._Proakis...` file)
* 
**Section:** *Chapter 14: Data Transmission in Fading Multipath Channels* / *Section 14.2: Channel Models for Time-Variant Multipath Channels* 


* 
**Details:** It defines the coherence time ($T_{\text{ct}}$) as a measure of the time interval over which the channel response changes very little. It explicitly states that a signal transmitted at two different time instants separated by less than $T_{\text{ct}}$ will be affected similarly by the channel.



---

### 2. The Jakes Model

The documents present **Jakes' model** as the standard way to define this smooth time correlation function via a Doppler power spectrum:

* **Document:** `_on_time_varying_channel.pdf`
* **Section:** *1.4.1.5 Global Channel Parameters* (Page 19, Example 1.9)
* 
**Details:** This example shows a popular channel model utilizing a **Jakes Doppler power profile**, where the continuous-time correlation function is modeled using the zeroth-order Bessel function of the first kind:



$$r_H^{(2)}(\Delta t) = \rho_H^2 J_0(2\pi \nu_{\text{max}} \Delta t)$$


.
This function strictly accounts for the smooth transition and eventual decorrelation between two instances separated by a time lag $\Delta t$.


* **Document:** `_ongoing_Proakis, Digital Communications, 5th ED.pdf`
* 
**Section:** *Section 14.2.1: Jakes' Model for the Doppler Power Spectrum* (Page 774 / Snippet 15/16) 


* 
**Details:** It explicitly models the autocorrelation of the time-varying transfer function using the zero-order Bessel function $J_0(2\pi f_m \tau)$ to track how the small-scale fading smoothly changes over time based on vehicle speed and maximum Doppler frequency ($f_m$).


* **Document:** `_ongoing__Andrea Goldsmith - Wireless Communications...`
* 
**Section:** *Section 3.2.1: Autocorrelation, Cross Correlation, and Power Spectral Density* 


* 
**Details:** Goldsmith details Clarke/Jakes' uniform scattering environment. She illustrates that the signal decorrelates at roughly $0.4\lambda$ but smoothly follows the $J_0(2\pi f_D \tau)$ curve up to that point.



---

### 3. The AR(1) / Gauss-Markov Model

Your Proakis text explicitly includes a problem formulating this time-varying channel using a first-order autoregressive process:

* **Document:** `_ongoing_Proakis, Digital Communications, 5th ED.pdf`
* 
**Section:** *Chapter 14 Problems* (Problem 14.12, Page 964) 


* **Details:** The text sets up the exact auto-regressive state equation to simulate how consecutive slots transition smoothly from one to the next:


$$h(m + 1) = \sqrt{1 - \alpha} \, h(m) + \alpha w(m + 1)$$


where $w(m)$ is an i.i.d. complex Gaussian random variable. This **Gauss-Markov model** is mathematically equivalent to a complex-valued AR(1) process used precisely to introduce temporal correlation between adjacent symbols or slots when $T_{\text{slot}}$ is small.