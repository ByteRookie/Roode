class SmartViewCard extends HTMLElement {
  setConfig(config) {
    this._config = config;
    if (this.shadowRoot) {
      this._update();
      return;
    }
    this.attachShadow({ mode: 'open' });
    const style = document.createElement('style');
    style.textContent = `
      :host {
        display: block;
        overflow: hidden;
        height: 100%;
        width: 100%;
      }
      .container {
        box-sizing: border-box;
        height: 100%;
        width: 100%;
        overflow-y: auto;
        overflow-x: hidden;
      }
    `;
    this._container = document.createElement('div');
    this._container.classList.add('container');
    this.shadowRoot.append(style, this._container);
    this._update();
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._config || !this.shadowRoot) return;
  }

  getCardSize() {
    return 3;
  }

  _update() {
    if (!this._container) return;
    const content = this._config.content || '';
    this._container.innerHTML = content;
  }
}
customElements.define('smart-view-card', SmartViewCard);
